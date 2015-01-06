////
// Plugin.cpp
//
//  Created on: Jan 27, 2014
//      Author: mklingen
////
#include "OctomapCollisionChecker.h"
#include <openrave/plugin.h>

using namespace OpenRAVE;
using namespace or_octomap;


OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    if (type == OpenRAVE::PT_CollisionChecker && interfacename == "or_octomap_checker")
    {
        return OpenRAVE::InterfaceBasePtr(new OctomapCollisionChecker(penv));
    }

    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_CollisionChecker].push_back("or_octomap_checker");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    return;
}
