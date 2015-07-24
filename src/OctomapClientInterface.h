////
// OctomapClientInterface.h
//
//  Created on: Jul 20, 2015
//      Author: dseredyn
////

#ifndef OCTOMAPCLIENTINTERFACE_H_
#define OCTOMAPCLIENTINTERFACE_H_

#include <openrave/openrave.h>
#include <octomap_server/OctomapServer.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/callback_queue.h>
#include <boost/thread/mutex.hpp>

namespace or_octomap
{
    class OctomapCollisionChecker;
    class OctomapClientInterface : public OpenRAVE::SensorSystemBase
    {
        public:
            OctomapClientInterface(OpenRAVE::EnvironmentBasePtr env, bool ros);
            virtual ~OctomapClientInterface();

            virtual bool SendCommand(std::ostream &os, std::istream &is);
            virtual void Reset();

            void SetEnabled(bool enabled);
            inline bool IsEnabled() { return m_isEnabled; }

            // Not implemented virtual functions.
            virtual void   AddRegisteredBodies (const std::vector< OpenRAVE::KinBodyPtr > &vbodies) { }
            virtual OpenRAVE::KinBody::ManageDataPtr  AddKinBody (OpenRAVE::KinBodyPtr pbody, OpenRAVE::XMLReadableConstPtr pdata) { return OpenRAVE::KinBody::ManageDataPtr(); }
            virtual bool RemoveKinBody(OpenRAVE::KinBodyPtr pbody) { return false; }
            virtual bool IsBodyPresent(OpenRAVE::KinBodyPtr pbody) { return false;}
            virtual bool EnableBody(OpenRAVE::KinBodyPtr pbody, bool bEnable) { return false; }
            virtual bool SwitchBody (OpenRAVE::KinBodyPtr pbody1, OpenRAVE::KinBodyPtr pbody2) { return false; }

            bool Enable(std::ostream &os, std::istream &i) { SetEnabled(true); return true;}
            bool Disable(std::ostream &os, std::istream &i) { SetEnabled(false); return true; }
            bool Update(std::ostream &os, std::istream &i);

            void Spin();
            bool UpdateOctomap();

            bool SetOcTree(std::ostream &os, std::istream &i);
            bool GetOcTree(std::ostream &os, std::istream &i);

        protected:
            void CreateFakeBody();
            void DestroyFakeBody();
            bool m_isEnabled;
            OctomapCollisionChecker* m_collisionChecker;
            ros::CallbackQueue m_queue;
            bool m_shouldExit;
            boost::mutex m_cloudQueueMutex;
            std::vector<sensor_msgs::PointCloud2ConstPtr> m_cloudQueue;
            bool m_isPaused;
            octomap_msgs::GetOctomap m_octomapMsg;
            octomap::OcTree *m_octree;
            bool m_ros;
            
    };

}
#endif
