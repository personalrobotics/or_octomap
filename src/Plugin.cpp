////
// Plugin.cpp
//
//  Created on: Jan 27, 2014
//      Author: mklingen
////
#include "OctomapInterface.h"
#include <openrave/plugin.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/param.h>
using namespace OpenRAVE;
using namespace or_octomap;


static char* argv[1] = {const_cast<char *>("or_octomap")};
static int argc = 1;
OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    if (type == OpenRAVE::PT_SensorSystem && interfacename == "or_octomap")
    {
        if (!ros::isInitialized())
        {
            ros::init(argc, argv, "or_octomap");
        }
        else
        {
            RAVELOG_DEBUG("Using existing ROS node '%s'\n", ros::this_node::getName().c_str());
        }



        std::map<std::string, std::string> remaps;
        remaps["/cloud_in"] = "/head/kinect2/sd/points";
        //fakeNode = new ros::NodeHandle("remap_node", remaps);

        ros::NodeHandle nodeHandle("~", remaps);

        // TODO: Figure out how to give user access to these parameters :(
        double resolution = 0.025;
        std::string frameID = "/map";
        double maxRange = 5.0;
        nodeHandle.param("resolution", resolution, resolution);
        nodeHandle.param("frame_id", frameID, frameID);
        nodeHandle.param("sensor_model/max_range", maxRange, maxRange);

        nodeHandle.setParam("resolution", resolution);
        nodeHandle.setParam("frame_id", frameID);
        nodeHandle.setParam("sensor_model/max_range", maxRange);


        return OpenRAVE::InterfaceBasePtr(new OctomapInterface(nodeHandle,  penv));
    }

    return OpenRAVE::InterfaceBasePtr();
}

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_SensorSystem].push_back("or_octomap");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    return;
}
