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
    OctomapClientInterface::OctomapClientInterface(OpenRAVE::EnvironmentBasePtr env, bool ros) :
        SensorSystemBase(env),
        m_shouldExit(false),
        m_octree(NULL),
        m_ros(ros)
    {
        RegisterCommand("Update", boost::bind(&OctomapClientInterface::Update, this, _1, _2),
                        "Update the octomap");

        RegisterCommand("Enable", boost::bind(&OctomapClientInterface::Enable, this, _1, _2),
                        "Begin collision testing octomap");
        RegisterCommand("Disable", boost::bind(&OctomapClientInterface::Disable, this, _1, _2),
                        "Stop collision testing octomap.");

        RegisterCommand("SetOcTree", boost::bind(&OctomapClientInterface::SetOcTree, this, _1, _2),
                        "Set the serialized OcTree message");

        RegisterCommand("GetOcTree", boost::bind(&OctomapClientInterface::GetOcTree, this, _1, _2),
                        "Get the serialized OcTree message");

        m_collisionChecker = NULL;
        if (ros)
        {
            boost::thread spinThread = boost::thread(boost::bind(&OctomapClientInterface::Spin, this));
        }
    }

    void OctomapClientInterface::Spin()
    {
        ros::Rate r(10);
        while(!m_shouldExit)
        {
            ros::spinOnce();
            r.sleep();
        }
    }
    
    OctomapClientInterface::~OctomapClientInterface()
    {
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
        if (m_ros)
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
        return false;
    }

    bool OctomapClientInterface::Update(std::ostream &os, std::istream &i)
    {
        return UpdateOctomap();
    }

    bool OctomapClientInterface::SetOcTree(std::ostream &os, std::istream &i)
    {
        i.get();
            SAFE_DELETE(m_octree);
        m_octree = static_cast<octomap::OcTree*> ( octomap::AbstractOcTree::read(i) );
        m_collisionChecker->SetTreeClone(m_octree);
        std::cout<<"m_octree size: "<<m_octree->size()<<std::endl;
        return true;
    }

    bool OctomapClientInterface::GetOcTree(std::ostream &os, std::istream &i)
    {
        m_octree->write(os);
        return true;
    }

}
