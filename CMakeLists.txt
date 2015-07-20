cmake_minimum_required(VERSION 2.8.3)
project(or_octomap)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  openrave_catkin
  sensor_msgs
  octomap_ros
  pcl_ros
  octomap_server
)
find_package(OpenRAVE REQUIRED)
find_package(octomap REQUIRED)

catkin_package()
include_directories(
  include
  ${OpenRAVE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(
    ${OpenRAVE_LIBRARY_DIRS}
)

set(SOURCES
 	src/OctomapInterface.cpp
 	src/OctomapInterface.h
 	src/OctomapCollisionChecker.cpp
 	src/OctomapCollisionChecker.h
 	src/FastAABBTriangleTest.h
    src/Plugin.cpp
    src/Plugin.h)

set(CLIENT_SOURCES
 	src/OctomapClientInterface.cpp
 	src/OctomapClientInterface.h
 	src/OctomapCollisionChecker.cpp
 	src/OctomapCollisionChecker.h
 	src/FastAABBTriangleTest.h
    src/ClientPlugin.cpp)

set(COLLISION_SOURCES
 	src/OctomapInterface.cpp
 	src/OctomapInterface.h
 	src/OctomapCollisionChecker.cpp
 	src/OctomapCollisionChecker.h
 	src/FastAABBTriangleTest.h
    src/CollisionCheckerPlugin.cpp)

openrave_plugin(${PROJECT_NAME}
    ${SOURCES}
)

openrave_plugin("${PROJECT_NAME}_client"
    ${CLIENT_SOURCES}
)

openrave_plugin("${PROJECT_NAME}_checker"
    ${COLLISION_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OPENRAVE_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)

target_link_libraries("${PROJECT_NAME}_client"
    ${catkin_LIBRARIES}
    ${OPENRAVE_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)

target_link_libraries("${PROJECT_NAME}_checker"
    ${catkin_LIBRARIES}
    ${OPENRAVE_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)
