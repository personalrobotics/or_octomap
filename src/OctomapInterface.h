////
// OctomapSensorSystem.h
//
//  Created on: Jan 24, 2014
//      Author: mklingen
//  Modified on: Jul 23, 2015
//      Author: dseredyn
////

#ifndef OCTOMAPSENSORSYSTEM_H_
#define OCTOMAPSENSORSYSTEM_H_

#include <openrave/openrave.h>
#include <octomap_server/OctomapServer.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/callback_queue.h>
#include <boost/thread/mutex.hpp>

namespace or_octomap
{
    class OctomapCollisionChecker;
    class OctomapInterface : public OpenRAVE::SensorSystemBase, public octomap_server::OctomapServer
    {
        public:
            OctomapInterface(ros::NodeHandle& nodeHandle, OpenRAVE::EnvironmentBasePtr env);
            virtual ~OctomapInterface();

            virtual bool SendCommand(std::ostream &os, std::istream &is);
            virtual void Reset();

            virtual void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

            void SetEnabled(bool enabled);
            inline bool IsEnabled() { return m_isEnabled; }

            octomap::OcTree* GetTree() { return m_octree; }


            // Not implemented virtual functions.
            virtual void   AddRegisteredBodies (const std::vector< OpenRAVE::KinBodyPtr > &vbodies) { }
            virtual OpenRAVE::KinBody::ManageDataPtr  AddKinBody (OpenRAVE::KinBodyPtr pbody, OpenRAVE::XMLReadableConstPtr pdata) { return OpenRAVE::KinBody::ManageDataPtr(); }
            virtual bool RemoveKinBody(OpenRAVE::KinBodyPtr pbody) { return false; }
            virtual bool IsBodyPresent(OpenRAVE::KinBodyPtr pbody) { return false;}
            virtual bool EnableBody(OpenRAVE::KinBodyPtr pbody, bool bEnable) { return false; }
            virtual bool SwitchBody (OpenRAVE::KinBodyPtr pbody1, OpenRAVE::KinBodyPtr pbody2) { return false; }

            bool TogglePause(std::ostream &os, std::istream &i) { m_isPaused = !m_isPaused; return true;}
            bool Enable(std::ostream &os, std::istream &i) { SetEnabled(true); return true;}
            bool Disable(std::ostream &os, std::istream &i) { SetEnabled(false); return true; }
            bool MaskObject(std::ostream &os, std::istream &i);
            bool UnmaskObject(std::ostream &os, std::istream &i);

            void Spin();
            void TestCollision();

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
            boost::mutex m_maskedObjectsMutex;
            std::vector<std::string> m_maskedObjects;
            OpenRAVE::KinBodyPtr m_sphere;
            std::string m_pointCloudTopic;
    };

}
#endif
