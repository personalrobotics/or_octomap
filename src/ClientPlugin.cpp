////
// ClientPlugin.cpp
//
//  Created on: Jul 20, 2015
//      Author: dseredyn
////
#include "OctomapClientInterface.h"
#include <openrave/plugin.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/param.h>
using namespace OpenRAVE;
using namespace or_octomap;


static char* argv[1] = {const_cast<char *>("or_octomap_client")};
static int argc = 1;
OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    if (type == OpenRAVE::PT_SensorSystem && interfacename == "or_octomap_client")
    {
        if (!ros::isInitialized())
        {
            ros::init(argc, argv, "or_octomap_client",  ros::init_options::AnonymousName);
        }
        else
        {
            RAVELOG_DEBUG("Using existing ROS node '%s'\n", ros::this_node::getName().c_str());
        }

        ros::NodeHandle nodeHandle("~");

        return OpenRAVE::InterfaceBasePtr(new OctomapClientInterface(nodeHandle,  penv));
    }

    return OpenRAVE::InterfaceBasePtr();
}

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_SensorSystem].push_back("or_octomap_client");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    return;
}
