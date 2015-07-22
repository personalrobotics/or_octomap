////
// OctomapCollisionChecker.cpp
//
//  Created on: Jan 24, 2014
//      Author: mklingen
////

#include "OctomapInterface.h"
#include "OctomapCollisionChecker.h"
#include "FastAABBTriangleTest.h"

namespace or_octomap
{
    
    OctomapCollisionChecker::OctomapCollisionChecker(OpenRAVE::EnvironmentBasePtr env) :
            CollisionCheckerBase(env), m_server(NULL), m_treeClone(NULL)
    {
        m_specialObject = "_OCTOMAP_MAP_";
    }

    OctomapCollisionChecker::OctomapCollisionChecker(OpenRAVE::EnvironmentBasePtr env, OpenRAVE::CollisionCheckerBasePtr wrappedChecker, OctomapInterface* server) :
            CollisionCheckerBase(env), m_wrappedChecker(wrappedChecker), m_server(server), m_treeClone(NULL)
    {
        m_specialObject = "_OCTOMAP_MAP_";

        // In this case there is no tree clone. We just use the tree from the server itself.
    }
    
    OctomapCollisionChecker::~OctomapCollisionChecker()
    {
        if(m_treeClone)
        {
            delete m_treeClone;
            m_treeClone = NULL;
        }
    }


    octomap::OcTree* OctomapCollisionChecker::GetTreeClone()
    {
        if (m_treeClone)
        {
            return m_treeClone;
        }

        if (m_server)
        {
            return m_server->GetTree();
        }

        return NULL;

    }

    void OctomapCollisionChecker::Clone(OpenRAVE::InterfaceBaseConstPtr preference, int cloningoptions)
    {
        CollisionCheckerBase::Clone(preference, cloningoptions);

        const OctomapCollisionChecker* otherChecker = dynamic_cast<const OctomapCollisionChecker*>(preference.get());

        if(!otherChecker)
        {
            return;
        }

        if(m_wrappedChecker)
        {
            m_wrappedChecker->Clone(otherChecker->m_wrappedChecker, cloningoptions);
        }
        else
        {
            m_wrappedChecker = OpenRAVE::RaveCreateCollisionChecker(GetEnv(), otherChecker->m_wrappedChecker->GetXMLId());
            m_wrappedChecker->Clone(otherChecker->m_wrappedChecker, cloningoptions);
        }

        m_server = otherChecker->m_server;

        if(m_server)
        {
            m_treeClone = new octomap::OcTree(*(m_server->GetTree()));
        }

        m_specialObject = otherChecker->m_specialObject;

    }

    int OctomapCollisionChecker::CollisionTestTriangleAABB(const OpenRAVE::Vector& t1, const OpenRAVE::Vector& t2, const OpenRAVE::Vector& t3, float x, float y, float z, float eX, float eY, float eZ)
    {
        static float boxCenter[3];
        boxCenter[0] = x;
        boxCenter[1] = y;
        boxCenter[2] = z;

        static float boxExtents[3];
        boxExtents[0] = eX;
        boxExtents[1] = eY;
        boxExtents[2] = eZ;

        static float triVerts[3][3];

        triVerts[0][0] = t1.x;
        triVerts[0][1] = t1.y;
        triVerts[0][2] = t1.z;

        triVerts[1][0] = t2.x;
        triVerts[1][1] = t2.y;
        triVerts[1][2] = t2.z;

        triVerts[2][0] = t3.x;
        triVerts[2][1] = t3.y;
        triVerts[2][2] = t3.z;


        return FastAABTriangleTest::triBoxOverlap(boxCenter, boxExtents, triVerts);
    }

    bool OctomapCollisionChecker::SetCollisionOptions(int collisionoptions)
    {
        return m_wrappedChecker->SetCollisionOptions(collisionoptions);
    }

    int OctomapCollisionChecker::GetCollisionOptions() const
    {
        return m_wrappedChecker->GetCollisionOptions();
    }

    void OctomapCollisionChecker::SetTolerance(OpenRAVE::dReal tolerance)
    {
        return m_wrappedChecker->SetTolerance(tolerance);
    }

    bool OctomapCollisionChecker::InitKinBody(OpenRAVE::KinBodyPtr pbody)
    {
        return m_wrappedChecker->InitKinBody(pbody);
    }

    bool OctomapCollisionChecker::CheckOctomapCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::CollisionReportPtr report)
    {
        std::vector<OpenRAVE::KinBodyConstPtr> vbodyExcluded;
        const std::vector< OpenRAVE::KinBody::LinkConstPtr > vlinkExcluded;
        return CheckOctomapCollision(pbody1, vbodyExcluded, vlinkExcluded, report);
    }

    bool OctomapCollisionChecker::CheckOctomapCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::KinBodyConstPtr pbody2, OpenRAVE::CollisionReportPtr report)
    {
        if(pbody1->GetName() != m_specialObject && pbody2->GetName() != m_specialObject)
        {
            return false;
        }
        else if(pbody1->GetName() == m_specialObject && pbody2->GetName() != m_specialObject)
        {
            return CheckOctomapCollision(pbody2, report);
        }
        else if(pbody1->GetName() != m_specialObject && pbody2->GetName() == m_specialObject)
        {
            return CheckOctomapCollision(pbody1, report);
        }
        else
        {
            return false;
        }

    }

    bool OctomapCollisionChecker::CheckOctomapCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report)
    {
        std::vector<OpenRAVE::KinBodyConstPtr> vbodyExcluded;
        const std::vector< OpenRAVE::KinBody::LinkConstPtr > vlinkExcluded;
        return CheckOctomapCollision(plink, vbodyExcluded, vlinkExcluded, report);
    }

    bool OctomapCollisionChecker::CheckOctomapCollision(OpenRAVE::KinBody::LinkConstPtr plink1, OpenRAVE::KinBody::LinkConstPtr plink2, OpenRAVE::CollisionReportPtr report)
    {
        // Octomap tree never has links
        return false;
    }

    bool OctomapCollisionChecker::CheckOctomapCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report)
    {
        if(pbody->GetName() != m_specialObject)
        {
            return false;
        }

        return CheckOctomapCollision(plink, report);
    }

    bool OctomapCollisionChecker::ContainsSpecialBody(const std::vector<OpenRAVE::KinBodyConstPtr>& bodies)
    {
        for(size_t i = 0; i < bodies.size(); i++)
        {
            if(bodies[i]->GetName() == m_specialObject)
            {
                return true;
            }
        }

        return false;
    }

    bool OctomapCollisionChecker::ContainsBody(const std::vector<OpenRAVE::KinBodyConstPtr>& links, OpenRAVE::KinBodyConstPtr link)
    {
        for(size_t i = 0; i < links.size(); i++)
        {
            if(links[i] == link)
            {
                return true;
            }
        }

        return false;
    }

    bool OctomapCollisionChecker::ContainsLink(const std::vector<OpenRAVE::KinBody::LinkConstPtr>& links, OpenRAVE::KinBody::LinkConstPtr link)
    {
        for(size_t i = 0; i < links.size(); i++)
        {
            if(links[i] == link)
            {
                return true;
            }
        }

        return false;
    }

    inline float min(float a, float b, float c)
    {
        return fmin(a, fmin(b, c));
    }

    inline float max(float a, float b, float c)
    {
        return fmax(a, fmax(b, c));
    }

    void OctomapCollisionChecker::ComputeTriangleAABB(const OpenRAVE::Vector& t1, const OpenRAVE::Vector& t2, const OpenRAVE::Vector& t3, OpenRAVE::AABB& aabb)
    {

        float minX = min(t1.x, t2.x, t3.x);
        float minY = min(t1.y, t2.y, t3.y);
        float minZ = min(t1.z, t2.z, t3.z);

        float maxX = max(t1.x, t2.x, t3.x);
        float maxY = max(t1.y, t2.y, t3.y);
        float maxZ = max(t1.z, t2.z, t3.z);


        aabb.pos.x = (minX + maxX) * 0.5f;
        aabb.pos.y = (minY + maxY) * 0.5f;
        aabb.pos.z = (minZ + maxZ) * 0.5f;

        aabb.extents.x = (maxX - minX) * 0.5f;
        aabb.extents.y = (maxY - minY) * 0.5f;
        aabb.extents.z = (maxZ - minZ) * 0.5f;

    }

    int  OctomapCollisionChecker::CollisionTestTriangle(const OpenRAVE::Vector& t1, const OpenRAVE::Vector& t2, const OpenRAVE::Vector& t3, octomap::OcTree::leaf_bbx_iterator& it)
    {
        const octomap::point3d& point = it.getCoordinate();
        double resolution = it.getSize();

        int check = CollisionTestTriangleAABB(t1, t2, t3, point.x(), point.y(), point.z(), resolution, resolution, resolution);
        if(check)
        {
            return check;
        }
        return 0;
    }

    bool OctomapCollisionChecker::CheckOctomapCollision(OpenRAVE::KinBody::LinkConstPtr plink, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report)
    {

        //TODO: Fill report!
        if(ContainsSpecialBody(vbodyexcluded))
        {
            return false;
        }

        OpenRAVE::AABB aabb = plink->ComputeAABB();

        const OpenRAVE::TriMesh& mesh = plink->GetCollisionData();

        OpenRAVE::Transform globalTransform = plink->GetTransform();

        octomap::OcTree::leaf_bbx_iterator leafIterBegin = GetTreeClone()->begin_leafs_bbx(octomap::point3d(aabb.pos.x - aabb.extents.x, aabb.pos.y - aabb.extents.y, aabb.pos.z - aabb.extents.z),  octomap::point3d(aabb.pos.x + aabb.extents.x, aabb.pos.y + aabb.extents.y, aabb.pos.z + aabb.extents.z));
        octomap::OcTree::leaf_bbx_iterator endIter = GetTreeClone()->end_leafs_bbx();


        for(octomap::OcTree::leaf_bbx_iterator it = leafIterBegin; it!= endIter; it++)
        {
            if(it->getOccupancy() <= GetTreeClone()->getOccupancyThres())
            {
                continue;
            }

            for(size_t j = 0; j < mesh.indices.size() / 3; j++)
            {
                OpenRAVE::Vector v1 = globalTransform * mesh.vertices.at(mesh.indices.at(j * 3 + 0));
                OpenRAVE::Vector v2 = globalTransform * mesh.vertices.at(mesh.indices.at(j * 3 + 1));
                OpenRAVE::Vector v3 = globalTransform * mesh.vertices.at(mesh.indices.at(j * 3 + 2));

                bool collides = CollisionTestTriangle(v1, v2, v3, it);

                if(collides)
                {
                    return true;
                }
            }
        }



        return false;
    }

    bool OctomapCollisionChecker::CheckOctomapCollision(OpenRAVE::KinBodyConstPtr pbody, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report)
    {
        //TODO: Fill report!
        if(ContainsSpecialBody(vbodyexcluded))
        {
            return false;
        }

        if(pbody->GetName() != m_specialObject)
        {
            OpenRAVE::AABB aabb = pbody->ComputeAABB();

            octomap::point3d minPoint = octomap::point3d(aabb.pos.x - aabb.extents.x, aabb.pos.y - aabb.extents.y, aabb.pos.z - aabb.extents.z);
            octomap::point3d maxPoint = octomap::point3d(aabb.pos.x + aabb.extents.x, aabb.pos.y + aabb.extents.y, aabb.pos.z + aabb.extents.z);

            octomap::OcTree::leaf_bbx_iterator it = GetTreeClone()->begin_leafs_bbx(minPoint, maxPoint);
            bool foundOccupied = false;
            while(it != GetTreeClone()->end_leafs_bbx())
            {
                if(it->getOccupancy() > GetTreeClone()->getOccupancyThres())
                {
                   foundOccupied = true;
                   break;
                }
                it++;
            }

            if(!foundOccupied) return false;

            for(size_t i = 0; i < pbody->GetLinks().size(); i++)
            {
                //ros::Time startLinkTime = ros::Time::now();
                OpenRAVE::KinBody::LinkConstPtr  link = pbody->GetLinks().at(i);

                if(ContainsLink(vlinkexcluded, link))
                {
                    continue;
                }

                bool collides = CheckOctomapCollision(link, vbodyexcluded, vlinkexcluded, report);

                if(collides)
                {
                    return true;
                }
            }
        }
        else
        {
            std::vector<OpenRAVE::KinBodyPtr> bodies;
            GetEnv()->GetBodies(bodies);

            for(size_t i = 0; i < bodies.size(); i++)
            {
                OpenRAVE::KinBodyConstPtr body = bodies.at(i);

                if(body->GetName() == m_specialObject || ContainsBody(vbodyexcluded, body))
                {
                    continue;
                }

                bool collidesBody = CheckOctomapCollision(body, vbodyexcluded, vlinkexcluded, report);

                if(collidesBody)
                {
                    return true;
                }
            }
        }
        return false;
    }

    bool OctomapCollisionChecker::CheckOctomapCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report)
    {
        // Octomap never has links
        return false;
    }

    bool OctomapCollisionChecker::CheckOctomapCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report)
    {
        if(pbody->GetName() != m_specialObject)
        {
            return false;
        }

        return CheckOctomapCollision(ray, report);
    }

    bool OctomapCollisionChecker::CheckOctomapCollision(const OpenRAVE::RAY &ray, OpenRAVE::CollisionReportPtr report)
    {
        octomap::point3d origin;
        origin(0) = ray.pos.x;
        origin(1) = ray.pos.y;
        origin(2) = ray.pos.z;

        octomap::point3d dir;
        dir(0) = ray.dir.x;
        dir(1) = ray.dir.y;
        dir(2) = ray.dir.z;

        octomap::point3d end;

        dir.normalize();

        // TODO: Fill report.
        bool collision = GetTreeClone()->castRay(origin, dir, end, true, ray.dir.lengthsqr3());
        report->minDistance = (end - origin).norm();

        return collision;
    }


    bool OctomapCollisionChecker::CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::CollisionReportPtr report)
    {
        bool collides = m_wrappedChecker->CheckCollision(pbody1, report);
        collides = collides || CheckOctomapCollision(pbody1, report);
        return collides;
    }

    bool OctomapCollisionChecker::CheckCollision(OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::KinBodyConstPtr pbody2, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(pbody1, pbody2, report) || CheckOctomapCollision(pbody1, pbody2, report);
    }

    bool OctomapCollisionChecker::CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(plink, report) || CheckOctomapCollision(plink, report);
    }

    bool OctomapCollisionChecker::CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink1, OpenRAVE::KinBody::LinkConstPtr plink2, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(plink1, plink2, report) || CheckOctomapCollision(plink1, plink2, report);
    }

    bool OctomapCollisionChecker::CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(plink, pbody, report) || CheckOctomapCollision(plink, pbody, report);
    }

    bool OctomapCollisionChecker::CheckCollision(OpenRAVE::KinBody::LinkConstPtr plink, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(plink, vbodyexcluded, vlinkexcluded, report) || CheckOctomapCollision(plink, vbodyexcluded, vlinkexcluded, report);
    }

    bool OctomapCollisionChecker::CheckCollision(OpenRAVE::KinBodyConstPtr pbody, const std::vector< OpenRAVE::KinBodyConstPtr > &vbodyexcluded, const std::vector< OpenRAVE::KinBody::LinkConstPtr > &vlinkexcluded, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(pbody, vbodyexcluded, vlinkexcluded, report) || CheckOctomapCollision(pbody, vbodyexcluded, vlinkexcluded, report);
    }

    bool OctomapCollisionChecker::CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(ray, plink, report) || CheckOctomapCollision(ray, plink, report);
    }

    bool OctomapCollisionChecker::CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(ray, pbody, report)|| CheckOctomapCollision(ray, pbody, report);
    }

    bool OctomapCollisionChecker::CheckCollision(const OpenRAVE::RAY &ray, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckCollision(ray, report) || CheckOctomapCollision(ray, report);
    }

    bool OctomapCollisionChecker::CheckStandaloneSelfCollision(OpenRAVE::KinBodyConstPtr body, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckStandaloneSelfCollision(body, report);
    }

    bool OctomapCollisionChecker::CheckStandaloneSelfCollision(OpenRAVE::KinBody::LinkConstPtr body, OpenRAVE::CollisionReportPtr report)
    {
        return m_wrappedChecker->CheckStandaloneSelfCollision(body, report);
    }

    bool OctomapCollisionChecker::InitEnvironment()
    {
        return m_wrappedChecker->InitEnvironment();
    }

    void OctomapCollisionChecker::DestroyEnvironment()
    {
        return m_wrappedChecker->DestroyEnvironment();
    }


    void OctomapCollisionChecker::RemoveKinBody(OpenRAVE::KinBodyPtr body)
    {
        m_wrappedChecker->RemoveKinBody(body);
    }


    void OctomapCollisionChecker::GetNodesColliding(const std::string& objectName, std::vector<octomap::OcTreeNode*>& nodes)
    {
        OpenRAVE::KinBodyPtr pbody = GetEnv()->GetKinBody(objectName);

        if(!pbody.get())
        {
            ROS_WARN("Unable to find object %s\n", objectName.c_str());
            return;
        }
        else
        {
            ROS_INFO("Masking...\n");
        }

        if (GetTreeClone()->size() == 0)
        {
            ROS_INFO("Empty tree...\n");
            return;
        }

        OpenRAVE::AABB aabb = pbody->ComputeAABB();
        float halfResolution = GetTreeClone()->getResolution() * 1.5f;

        for(size_t i = 0; i < pbody->GetLinks().size(); i++)
        {
            OpenRAVE::KinBody::LinkConstPtr  plink = pbody->GetLinks().at(i);


            aabb = plink->ComputeAABB();
            aabb.extents *= 1.25f;

            octomap::point3d minPoint = octomap::point3d(aabb.pos.x - aabb.extents.x, aabb.pos.y - aabb.extents.y, aabb.pos.z - aabb.extents.z);
            octomap::point3d maxPoint = octomap::point3d(aabb.pos.x + aabb.extents.x, aabb.pos.y + aabb.extents.y, aabb.pos.z + aabb.extents.z);

            octomap::OcTree::leaf_bbx_iterator it = GetTreeClone()->begin_leafs_bbx(minPoint, maxPoint);
            bool foundOccupied = false;
            while(it != GetTreeClone()->end_leafs_bbx())
            {
                if(it->getOccupancy() > GetTreeClone()->getOccupancyThres())
                {
                   foundOccupied = true;
                   break;
                }
                it++;
            }

            if(!foundOccupied) continue;

            const std::vector<  OpenRAVE::KinBody::Link::GeometryPtr > geoms = plink->GetGeometries();

            OpenRAVE::Transform globalTransform = plink->GetTransform();

            octomap::OcTree::leaf_bbx_iterator leafIterBegin = GetTreeClone()->begin_leafs_bbx(octomap::point3d(aabb.pos.x - aabb.extents.x, aabb.pos.y - aabb.extents.y, aabb.pos.z - aabb.extents.z),  octomap::point3d(aabb.pos.x + aabb.extents.x, aabb.pos.y + aabb.extents.y, aabb.pos.z + aabb.extents.z));
            octomap::OcTree::leaf_bbx_iterator endIter = GetTreeClone()->end_leafs_bbx();

            for(octomap::OcTree::leaf_bbx_iterator it = leafIterBegin; it!= endIter; it++)
            {

                const octomap::point3d& point = it.getCoordinate();
                float size = it.getSize() * 1.5f;

                if(it->getOccupancy() > GetTreeClone()->getOccupancyThres())
                {
                    for (std::vector<  OpenRAVE::KinBody::Link::GeometryPtr >::const_iterator git = geoms.begin(); git != geoms.end(); git++)
                    {
                        OpenRAVE::Transform localTransform = (*git)->GetTransform();
                        if ((*git)->GetType() == OpenRAVE::GT_Sphere)
                        {
                            double radius = (*git)->GetSphereRadius();
                            OpenRAVE::Vector pos = localTransform * OpenRAVE::Vector();
                            double dist_2 = (point.x()-pos[0]) * (point.x()-pos[0]) + (point.y()-pos[1]) * (point.y()-pos[1]) + (point.z()-pos[2]) * (point.z()-pos[2]);
                            if (dist_2 <= (radius+size)*(radius+size))
                            {
                                nodes.push_back(GetTreeClone()->search(it.getKey()));
                            }
                        }
                        else if ((*git)->GetType() == OpenRAVE::GT_Cylinder)
                        {
                            OpenRAVE::Transform T_W_G = globalTransform * localTransform;
                            OpenRAVE::Transform T_G_W = T_W_G.inverse();
                            OpenRAVE::Vector point_G = T_G_W * OpenRAVE::Vector(point.x(), point.y(), point.z());
                            double dist_xy = sqrt(point_G[0] * point_G[0] + point_G[1] * point_G[1]);
                            double radius = (*git)->GetCylinderRadius();
                            double height = (*git)->GetCylinderHeight();
                            if (dist_xy < size + radius && point_G[2] > -size && point_G[2] < height + size)
                            {
                                nodes.push_back(GetTreeClone()->search(it.getKey()));
                            }
                        }
                        else if ((*git)->GetType() == OpenRAVE::GT_Box)
                        {
                            OpenRAVE::Transform T_W_G = globalTransform * localTransform;
                            OpenRAVE::Transform T_G_W = T_W_G.inverse();
                            OpenRAVE::Vector point_G = T_G_W * OpenRAVE::Vector(point.x(), point.y(), point.z());
                            OpenRAVE::Vector box = (*git)->GetBoxExtents();
                            if (point_G[0] > -box[0]-size && point_G[0] < box[0]+size &&
                                point_G[1] > -box[1]-size && point_G[1] < box[1]+size &&
                                point_G[2] > -box[2]-size && point_G[2] < box[2]+size)
                            {
                                nodes.push_back(GetTreeClone()->search(it.getKey()));
                            }
                        }
                        else
                        {
                            const OpenRAVE::TriMesh& mesh = (*git)->GetCollisionMesh();
                            for(size_t j = 0; j < mesh.indices.size() / 3; j++)
                            {
                                OpenRAVE::Vector t1 = globalTransform * localTransform * mesh.vertices.at(mesh.indices.at(j * 3 + 0));
                                OpenRAVE::Vector t2 = globalTransform * localTransform * mesh.vertices.at(mesh.indices.at(j * 3 + 1));
                                OpenRAVE::Vector t3 = globalTransform * localTransform * mesh.vertices.at(mesh.indices.at(j * 3 + 2));

                                int check = CollisionTestTriangleAABB(t1, t2, t3, point.x(), point.y(), point.z(), size, size, size);

                                if(check)
                                {
                                    nodes.push_back(GetTreeClone()->search(it.getKey()));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    bool OctomapCollisionChecker::MaskObject(std::string name)
    {
        std::vector<octomap::OcTreeNode*> nodes;
        GetNodesColliding(name, nodes);

        ROS_INFO("Got %lu colliding nodes\n", nodes.size());
        for(std::vector<octomap::OcTreeNode*>::iterator it = nodes.begin(); it != nodes.end(); it++)
        {
           if(*it)
           {
               for(int i = 0; i < 10; i++)
                   GetTreeClone()->integrateMiss(*it);
           }
        }

        return true;
    }

} /* namespace perception_utils */
