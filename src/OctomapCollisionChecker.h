////
// OctomapCollisionChecker.h
//
//  Created on: Jan 24, 2014
//      Author: mklingen
////

#ifndef OCTOMAPCOLLISIONCHECKER_H_
#define OCTOMAPCOLLISIONCHECKER_H_

#include <octomap/OcTree.h>
#include <string>
#include <openrave/openrave.h>
#include <openrave/collisionchecker.h>
#include <ros/time.h>


namespace or_octomap
{
    class OctomapInterface;
    class OctomapCollisionChecker : public OpenRAVE::CollisionCheckerBase
    {
        public:
            OctomapCollisionChecker(OpenRAVE::EnvironmentBasePtr env);
            OctomapCollisionChecker(OpenRAVE::EnvironmentBasePtr env, OpenRAVE::CollisionCheckerBasePtr wrappedChecker, OctomapInterface* server);
            virtual ~OctomapCollisionChecker();
            virtual void  Clone (OpenRAVE::InterfaceBaseConstPtr preference, int cloningoptions);
            virtual bool SetCollisionOptions(int collisionoptions);
            virtual int GetCollisionOptions() const;
            virtual void SetTolerance(OpenRAVE::dReal tolerance);
            virtual bool InitKinBody(OpenRAVE::KinBodyPtr pbody);
            virtual bool CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::KinBodyConstPtr pbody2, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink1, OpenRAVE::KinBody::LinkConstPtr plink2, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(OpenRAVE::KinBodyConstPtr pbody, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckStandaloneSelfCollision(OpenRAVE::KinBodyConstPtr body, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckStandaloneSelfCollision(OpenRAVE::KinBody::LinkConstPtr body, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool InitEnvironment();
            virtual void DestroyEnvironment();
            virtual void RemoveKinBody(OpenRAVE::KinBodyPtr body);

            virtual bool CheckOctomapCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckOctomapCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::KinBodyConstPtr pbody2, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckOctomapCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckOctomapCollision(OpenRAVE::KinBody::LinkConstPtr plink1, OpenRAVE::KinBody::LinkConstPtr plink2, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckOctomapCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckOctomapCollision(OpenRAVE::KinBody::LinkConstPtr plink, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckOctomapCollision(OpenRAVE::KinBodyConstPtr pbody, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckOctomapCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckOctomapCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());
            virtual bool CheckOctomapCollision(const OpenRAVE::RAY &ray, OpenRAVE::CollisionReportPtr report=OpenRAVE::CollisionReportPtr());

            inline OpenRAVE::CollisionCheckerBasePtr GetWrappedChecker() { return m_wrappedChecker; }
            inline void SetWrappedChecker(OpenRAVE::CollisionCheckerBasePtr checker) { m_wrappedChecker = checker;}

            inline void SetInterface(OctomapInterface* interface) { m_server = interface; }
            inline OctomapInterface* GetInterface() { return m_server; }

            inline void SetTreeClone(octomap::OcTree* treeClone) { m_treeClone = treeClone; }
            octomap::OcTree* GetTreeClone();

            void GetNodesColliding(const std::string& objectName, std::vector<octomap::OcTreeNode*>& nodes);
            bool MaskObject(std::string name);

        protected:
            void ComputeTriangleAABB(const OpenRAVE::Vector& t1, const OpenRAVE::Vector& t2, const OpenRAVE::Vector& t3, OpenRAVE::AABB& aabb);
            bool ContainsSpecialBody(const std::vector<OpenRAVE::KinBodyConstPtr>& bodies);
            int CollisionTestTriangle(const OpenRAVE::Vector& t1, const OpenRAVE::Vector& t2, const OpenRAVE::Vector& t3, octomap::OcTree::leaf_bbx_iterator& it);
            int CollisionTestTriangleAABB(const OpenRAVE::Vector& t1, const OpenRAVE::Vector& t2, const OpenRAVE::Vector& t3, float x, float y, float z, float eX, float eY, float eZ);
            bool ContainsLink(const std::vector<OpenRAVE::KinBody::LinkConstPtr>& links, OpenRAVE::KinBody::LinkConstPtr link);
            bool ContainsBody(const std::vector<OpenRAVE::KinBodyConstPtr>& links, OpenRAVE::KinBodyConstPtr link);
            std::string m_specialObject;
            OpenRAVE::CollisionCheckerBasePtr m_wrappedChecker;
            OctomapInterface* m_server;
            octomap::OcTree* m_treeClone;
            ros::Duration triangleTime;
            ros::Duration bodyTime;
            ros::Duration linkTime;
            ros::Duration testTime;
            ros::Duration geomTime;

    };

} /* namespace perception_utils */
#endif /* OCTOMAPCOLLISIONCHECKER_H_ */
