////
// OctomapSensorSystem.cpp
//
//  Created on: Jan 24, 2014
//      Author: mklingen
//  Modified on: Jul 23, 2015
//      Author: dseredyn
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
        SensorSystemBase(env),
        OctomapServer(nodeHandle),
        m_shouldExit(false),
        m_pointCloudTopic("cloud_in")
    {

        m_isPaused = false;

        nodeHandle.param("point_cloud_topic", m_pointCloudTopic, m_pointCloudTopic);

        m_pointCloudSub->unsubscribe();
        m_pointCloudSub->subscribe(nodeHandle, m_pointCloudTopic, 5);

        ROS_INFO("Frame ID: %s", m_worldFrameId.c_str());
        ROS_INFO("Topic: %s", m_pointCloudSub->getTopic().c_str());

        RegisterCommand("Enable", boost::bind(&OctomapInterface::Enable, this, _1, _2),
                        "Begin collision testing octomap");
        RegisterCommand("Disable", boost::bind(&OctomapInterface::Disable, this, _1, _2),
                        "Stop collision testing octomap.");
        RegisterCommand("Mask", boost::bind(&OctomapInterface::MaskObject, this, _1, _2),
                        "Mask an object out of the octomap");
        RegisterCommand("Unmask", boost::bind(&OctomapInterface::UnmaskObject, this, _1, _2),
                        "Remove the mask for object");
        RegisterCommand("TogglePause", boost::bind(&OctomapInterface::TogglePause, this, _1, _2),
                        "Toggles the octomap to being paused/unpaused for collecting data");

        RegisterCommand("UpdateAllMasks", boost::bind(&OctomapInterface::UpdateAllMasks, this, _1, _2),
                        "Apply the mask for all objects to filter them out of the octomap");
        RegisterCommand("UpdateMask", boost::bind(&OctomapInterface::UpdateMask, this, _1, _2),
                        "Update the mask of the object");


        RegisterCommand("GetOcTree", boost::bind(&OctomapInterface::GetOcTree, this, _1, _2),
                        "Get the serialized OcTree");

        m_collisionChecker = NULL;
        boost::thread spinThread = boost::thread(boost::bind(&OctomapInterface::Spin, this));
        //boost::thread(boost::bind(&OctomapInterface::TestCollision, this));

        m_sphere = OpenRAVE::RaveCreateKinBody(env);
        std::vector< OpenRAVE::Vector > spheres;
        spheres.push_back( OpenRAVE::Vector(0,0,0,0.04) );
        m_sphere->InitFromSpheres(spheres, true);
        m_sphere->SetName("octomap_server_probe_sphere");
    }

    // this method is copied from OctomapServer.cpp and modified
    void OctomapInterface::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& ground, const PCLPointCloud& nonground){
        octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

        if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
            || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
        {
            ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
        }

        //
        // add mask information to the point cloud
        //
        OpenRAVE::EnvironmentBasePtr  penv = GetEnv();

        // create a vector of masked bodies
        std::vector<OpenRAVE::KinBodyPtr > maskedBodies;
        {
            boost::mutex::scoped_lock(m_maskedObjectsMutex);
            for (std::vector<std::string >::const_iterator o_it = m_maskedObjects.begin(); o_it != m_maskedObjects.end(); o_it++)
            {
                KinBodyPtr pbody = penv->GetKinBody(*o_it);
                if (pbody.get() != NULL)
                {
                    maskedBodies.push_back( pbody );
                }
            }
        }

        int numBodies = maskedBodies.size();

        // instead of direct scan insertion, compute update to filter ground:
        octomap::KeySet free_cells, occupied_cells;
        // insert ground points only as free:
        for (PCLPointCloud::const_iterator it = ground.begin(); it != ground.end(); ++it){
            octomap::point3d point(it->x, it->y, it->z);
            // maxrange check
            if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
                point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            }

            // only clear space (ground points)
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }

            octomap::OcTreeKey endKey;
            if (m_octree->coordToKeyChecked(point, endKey)){
                updateMinKey(endKey, m_updateBBXMin);
                updateMaxKey(endKey, m_updateBBXMax);
            } else{
                ROS_ERROR_STREAM("Could not generate Key for endpoint "<<point);
            }
        }

        penv->Add(m_sphere, true);
        OpenRAVE::Transform tr;

        // all other points: free on ray, occupied on endpoint:
        for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it){
            octomap::point3d point(it->x, it->y, it->z);
            tr.trans.x = it->x;
            tr.trans.y = it->y;
            tr.trans.z = it->z;
            m_sphere->SetTransform(tr);
            bool maskedHit = false;
            for (int b_idx = 0; b_idx < numBodies; b_idx++)
            {
                if (penv->CheckCollision(maskedBodies[b_idx], m_sphere))
                {
                    maskedHit = true;
                    break;
                }
            }

            if (maskedHit)
            {
                // free cells
                if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                }
            }
            else if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) { // maxrange check

                // free cells
                if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                }
                // occupied endpoint
                octomap::OcTreeKey key;
                if (m_octree->coordToKeyChecked(point, key)){
                    occupied_cells.insert(key);

                    updateMinKey(key, m_updateBBXMin);
                    updateMaxKey(key, m_updateBBXMax);
                }
            } else {// ray longer than maxrange:;
                octomap::point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
                if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());

                    octomap::OcTreeKey endKey;
                    if (m_octree->coordToKeyChecked(new_end, endKey)){
                        updateMinKey(endKey, m_updateBBXMin);
                        updateMaxKey(endKey, m_updateBBXMax);
                    } else{
                        ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
                    }
                }
            }
        }
        penv->Remove(m_sphere);

        // mark free cells only if not seen occupied in this cloud
        for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
            if (occupied_cells.find(*it) == occupied_cells.end()){
                m_octree->updateNode(*it, false);
            }
        }

        // now mark all occupied cells:
        for (octomap::KeySet::iterator it = occupied_cells.begin(), end=free_cells.end(); it!= end; it++) {
            m_octree->updateNode(*it, true);
        }

        // TODO: eval lazy+updateInner vs. proper insertion
        // non-lazy by default (updateInnerOccupancy() too slow for large maps)
        //m_octree->updateInnerOccupancy();
        octomap::point3d minPt, maxPt;
        ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

        // TODO: snap max / min keys to larger voxels by m_maxTreeDepth
      //   if (m_maxTreeDepth < 16)
      //   {
      //      OcTreeKey tmpMin = getIndexKey(m_updateBBXMin, m_maxTreeDepth); // this should give us the first key at depth m_maxTreeDepth that is smaller or equal to m_updateBBXMin (i.e. lower left in 2D grid coordinates)
      //      OcTreeKey tmpMax = getIndexKey(m_updateBBXMax, m_maxTreeDepth); // see above, now add something to find upper right
      //      tmpMax[0]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
      //      tmpMax[1]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
      //      tmpMax[2]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
      //      m_updateBBXMin = tmpMin;
      //      m_updateBBXMax = tmpMax;
      //   }

        // TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
        minPt = m_octree->keyToCoord(m_updateBBXMin);
        maxPt = m_octree->keyToCoord(m_updateBBXMax);
        ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
        ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

        if (m_compressMap)
            m_octree->prune();
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
        ros::Rate r(10);
        while(!m_shouldExit)
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    OctomapInterface::~OctomapInterface()
    {
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

        {
            boost::mutex::scoped_lock(m_maskedObjectsMutex);
            m_maskedObjects.push_back(objectName);
        }

        bool toReturn = m_collisionChecker->MaskObject(objectName);
        publishAll();

        return toReturn;
    }

    bool OctomapInterface::UnmaskObject(std::ostream &os, std::istream &i)
    {

        std::string objectName;
        i >> objectName;

        ROS_INFO("Unmasking object %s\n", objectName.c_str());

        if(objectName == "" || !IsEnabled())
        {
            return false;
        }

        {
            boost::mutex::scoped_lock(m_maskedObjectsMutex);
            for (std::vector<std::string >::iterator o_it = m_maskedObjects.begin(); o_it != m_maskedObjects.end(); o_it++)
            {
                if ( (*o_it) == objectName )
                {
                    m_maskedObjects.erase(o_it);
                    return true;
                }
            }
        }

        return false;
    }

    bool OctomapInterface::UpdateAllMasks(std::ostream &os, std::istream &i)
    {
        // copy the vector of names
        std::vector<std::string > maskedObjects;
        {
            boost::mutex::scoped_lock(m_maskedObjectsMutex);

            maskedObjects.reserve(m_maskedObjects.size());
            for (std::vector<std::string >::iterator o_it = m_maskedObjects.begin(); o_it != m_maskedObjects.end(); o_it++)
            {
                maskedObjects.push_back(*o_it);
            }
        }

        for (std::vector<std::string >::iterator o_it = maskedObjects.begin(); o_it != maskedObjects.end(); o_it++)
        {
            m_collisionChecker->MaskObject(*o_it);
        }
        publishAll();

        return true;
    }

    bool OctomapInterface::UpdateMask(std::ostream &os, std::istream &i)
    {
        std::string objectName;
        i >> objectName;

        // check if the name is in the vector of names
        {
            bool nameFound = false;
            boost::mutex::scoped_lock(m_maskedObjectsMutex);
            for (std::vector<std::string >::iterator o_it = m_maskedObjects.begin(); o_it != m_maskedObjects.end(); o_it++)
            {
                if (objectName == *o_it)
                {
                    nameFound = true;
                }
            }
            if (!nameFound)
            {
                return false;
            }
        }

        m_collisionChecker->MaskObject(objectName);
        publishAll();

        return true;
    }

    bool OctomapInterface::GetOcTree(std::ostream &os, std::istream &i)
    {
        m_octree->write(os);
        return true;
    }

}
