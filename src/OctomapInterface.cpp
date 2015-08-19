////
// OctomapSensorSystem.cpp
//
//  Created on: Jan 24, 2014
//      Author: mklingen
////

#include "OctomapInterface.h"
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
    
    OctomapInterface::OctomapInterface(ros::NodeHandle& nodeHandle, OpenRAVE::EnvironmentBasePtr env) :
        SensorSystemBase(env), OctomapServer(nodeHandle), m_shouldExit(false)
    {

        m_isPaused = false;
        m_pointCloudSub->unsubscribe();

        //delete m_pointCloudSub;
        delete m_tfPointCloudSub;

        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh,  "/head/kinect2/sd/points", 1, ros::TransportHints(), &m_queue);
        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 1, m_nh, ros::Duration(0.1));
        m_tfPointCloudSub->registerCallback(boost::bind(&OctomapInterface::InsertCloudWrapper, this, _1));

        ROS_INFO("Frame ID: %s", m_worldFrameId.c_str());
        ROS_INFO("Topic: %s", m_pointCloudSub->getTopic().c_str());

        RegisterCommand("Enable", boost::bind(&OctomapInterface::Enable, this, _1, _2),
                        "Begin collision testing octomap");
        RegisterCommand("Disable", boost::bind(&OctomapInterface::Disable, this, _1, _2),
                        "Stop collision testing octomap.");
        RegisterCommand("Mask", boost::bind(&OctomapInterface::MaskObject, this, _1, _2),
                        "Mask an object out of the octomap");
        RegisterCommand("TogglePause", boost::bind(&OctomapInterface::TogglePause, this, _1, _2),
                        "Toggles the octomap to being paused/unpaused for collecting data");



        m_collisionChecker = NULL;
        boost::thread spinThread = boost::thread(boost::bind(&OctomapInterface::Spin, this));
        //boost::thread(boost::bind(&OctomapInterface::TestCollision, this));
    }
    
    void OctomapInterface::InsertCloudWrapper(const sensor_msgs::PointCloud2::ConstPtr& cloud)
    {
        boost::mutex::scoped_lock(m_cloudQueueMutex);

        if(!m_isPaused)
        {
            m_cloudQueue.push_back(cloud);
        }
    }

    void OctomapInterface::InsertScans()
    {
        while(m_cloudQueue.size() > 0)
        {
            boost::mutex::scoped_lock(m_cloudQueueMutex);

            insertCloudCallback(m_cloudQueue.front());
            m_cloudQueue.erase(m_cloudQueue.begin());

        }
    }



    void OctomapInterface::TestCollision()
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

    void OctomapInterface::Spin()
    {
        ros::WallDuration timeout(0.1);
        while(!m_shouldExit)
        {
            m_queue.callOne(timeout);
            InsertScans();
        }
    }

    OctomapInterface::~OctomapInterface()
    {
        SetEnabled(false);
        m_shouldExit = true;
    }

    void OctomapInterface::SetEnabled(bool enabled)
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
            //new OctomapCollisionChecker(GetEnv(), GetEnv()->GetCollisionChecker(), this);
            m_collisionChecker->SetInterface(this);
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


    void OctomapInterface::DestroyFakeBody()
    {
        OpenRAVE::KinBodyPtr kinBody = GetEnv()->GetKinBody("_OCTOMAP_MAP_");

        if(kinBody.get())
        {
            GetEnv()->Remove(kinBody);
        }
    }

    void OctomapInterface::CreateFakeBody()
    {
        ROS_DEBUG("Creating fake body!");
        OpenRAVE::KinBodyPtr kinbody = RaveCreateKinBody(GetEnv(), "");
        kinbody->SetName("_OCTOMAP_MAP_");
        GetEnv()->Add(kinbody);
    }

    bool OctomapInterface::SendCommand(std::ostream &os, std::istream &is)
    {
        return SensorSystemBase::SendCommand(os, is);
    }

    void OctomapInterface::Reset()
    {
        std_srvs::EmptyRequest req;
        std_srvs::EmptyResponse res;
        resetSrv(req, res);
    }

    bool OctomapInterface::MaskObject(std::ostream &os, std::istream &i)
    {

        std::string objectName;
        i >> objectName;

        ROS_INFO("Masking object %s\n", objectName.c_str());

        if(objectName == "" || !IsEnabled())
        {
            return false;
        }


        bool toReturn = m_collisionChecker->MaskObject(objectName);
        publishAll();

        return toReturn;
    }




}
