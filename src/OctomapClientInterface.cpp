////
// OctomapClientInterface.cpp
//
//  Created on: Jul 20, 2015
//      Author: dseredyn
////

#include "OctomapClientInterface.h"
#include "OctomapCollisionChecker.h"
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
using namespace OpenRAVE;
using namespace octomap_server;

#define SAFE_DELETE(x) if((x)) { delete (x);  x = NULL; }

namespace or_octomap
{
    
    OctomapClientInterface::OctomapClientInterface(ros::NodeHandle& nodeHandle, OpenRAVE::EnvironmentBasePtr env) :
        SensorSystemBase(env), m_shouldExit(false), m_octomapClient(nodeHandle.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary")), m_octree(NULL)
    {
        RegisterCommand("Update", boost::bind(&OctomapClientInterface::Update, this, _1, _2),
                        "Update the octomap");

        RegisterCommand("Enable", boost::bind(&OctomapClientInterface::Enable, this, _1, _2),
                        "Begin collision testing octomap");
        RegisterCommand("Disable", boost::bind(&OctomapClientInterface::Disable, this, _1, _2),
                        "Stop collision testing octomap.");
        RegisterCommand("Mask", boost::bind(&OctomapClientInterface::MaskObject, this, _1, _2),
                        "Mask an object out of the octomap");

        m_collisionChecker = NULL;
    }
    
    void OctomapClientInterface::TestCollision()
    {
        float timey = 0.0f;
        bool lastCollisionState = false;
        while(!m_shouldExit)
        {
            timey += 0.01f;
            OpenRAVE::CollisionReportPtr report(new OpenRAVE::CollisionReport());
            OpenRAVE::KinBodyConstPtr fakeBody = GetEnv()->GetKinBody("_OCTOMAP_MAP_");
            OpenRAVE::KinBodyPtr fuze = GetEnv()->GetKinBody("fuze_bottle");
            if(fakeBody.get())
            {
                boost::mutex::scoped_lock(m_cloudQueueMutex);
                bool collides = GetEnv()->CheckCollision(fakeBody, report);

                ROS_INFO("Collides: %s", collides ? "true" : "false");

                OpenRAVE::Transform t;
                t.identity();

                float r = 0.1 * cos(timey * 0.1f);
                t.trans = OpenRAVE::Vector(r * sin(timey) + 0.5f, r * cos(timey), 0.0f);

                if(fuze.get())
                {
                    fuze->SetTransform(t);
                }

                lastCollisionState = collides;
            }

            usleep(10000);
        }
    }

    OctomapClientInterface::~OctomapClientInterface()
    {
        SetEnabled(false);
        m_shouldExit = true;
    }

    void OctomapClientInterface::SetEnabled(bool enabled)
    {
        ROS_INFO("Set enabled called!");
        m_isEnabled = enabled;

        if(enabled)
        {
            if(m_collisionChecker)
            {
                GetEnv()->SetCollisionChecker(m_collisionChecker->GetWrappedChecker());
                SAFE_DELETE(m_collisionChecker);
            }

            OpenRAVE::CollisionCheckerBasePtr collisionCheckerBase = RaveCreateCollisionChecker(GetEnv(), "or_octomap_checker");

            m_collisionChecker = dynamic_cast<OctomapCollisionChecker*>(collisionCheckerBase.get());
            UpdateOctomap();
            m_collisionChecker->SetTreeClone(m_octree);
            m_collisionChecker->SetWrappedChecker(GetEnv()->GetCollisionChecker());
            CreateFakeBody();
            GetEnv()->SetCollisionChecker(collisionCheckerBase);
        }
        else
        {
            if(m_collisionChecker)
            {
                GetEnv()->SetCollisionChecker(m_collisionChecker->GetWrappedChecker());
                SAFE_DELETE(m_collisionChecker);
                DestroyFakeBody();
            }
        }
    }

    void OctomapClientInterface::DestroyFakeBody()
    {
        OpenRAVE::KinBodyPtr kinBody = GetEnv()->GetKinBody("_OCTOMAP_MAP_");

        if(kinBody.get())
        {
            GetEnv()->Remove(kinBody);
        }
    }

    void OctomapClientInterface::CreateFakeBody()
    {
        ROS_DEBUG("Creating fake body!");
        OpenRAVE::KinBodyPtr kinbody = RaveCreateKinBody(GetEnv(), "");
        kinbody->SetName("_OCTOMAP_MAP_");
        GetEnv()->Add(kinbody);
    }

    bool OctomapClientInterface::SendCommand(std::ostream &os, std::istream &is)
    {
        return SensorSystemBase::SendCommand(os, is);
    }

    void OctomapClientInterface::Reset()
    {
    }

    bool OctomapClientInterface::UpdateOctomap()
    {
        if (ros::service::call("/octomap_binary", m_octomapMsg))
        {
            SAFE_DELETE(m_octree);
            m_octree = octomap_msgs::binaryMsgToMap(m_octomapMsg.response.map);
            m_collisionChecker->SetTreeClone(m_octree);
        }
        else
        {
            std::cout << "OctomapClientInterface::UpdateOctomap failed" << std::endl;
            return false;
        }
        
        return true;
    }

    bool OctomapClientInterface::Update(std::ostream &os, std::istream &i)
    {
        return UpdateOctomap();
    }

    bool OctomapClientInterface::MaskObject(std::ostream &os, std::istream &i)
    {

        std::string objectName;
        i >> objectName;

        ROS_INFO("Masking object %s\n", objectName.c_str());

        if(objectName == "" || !IsEnabled())
        {
            return false;
        }


        bool toReturn = m_collisionChecker->MaskObject(objectName);

        return toReturn;
    }

}
