////
// TestOrOctomap.cpp
//
//  Created on: Jan 28, 2014
//      Author: mklingen
////

#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

using namespace OpenRAVE;
using namespace std;
void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv, viewername);
    BOOST_ASSERT(!!viewer);
    // attach it to the environment:
    penv->Add(viewer);
    // finally call the viewer's infinite loop (this is why a separate thread is needed)
    bool showgui = true;
    viewer->main(showgui);
}

int main(int argc, char ** argv)
{
    //int num = 1;
    string scenefilename = "/opt/pr/pr_ordata/ordata/objects/household/fuze_bottle.kinbody.xml";
    string viewername = "or_rviz";
    // parse the command line options
    int i = 1;
    while (i < argc)
    {
        if ((strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "-?") == 0)
                || (strcmp(argv[i], "/?") == 0)
                || (strcmp(argv[i], "--help") == 0)
                || (strcmp(argv[i], "-help") == 0))
        {
            RAVELOG_INFO(
                    "orloadviewer [--num n] [--scene filename] viewername\n");
            return 0;
        }
        else if (strcmp(argv[i], "--scene") == 0)
        {
            scenefilename = argv[i + 1];
            i += 2;
        }
        else
            break;
    }
    if (i < argc)
    {
        viewername = argv[i++];
    }
    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
    RaveLoadPlugin("or_octomap");
    OpenRAVE::SensorSystemBasePtr sensors = RaveCreateSensorSystem(penv, "or_octomap");
    string out;
    sensors->SendCommand(out, "Enable");
    RaveSetDebugLevel(Level_Debug);
    boost::thread thviewer(boost::bind(SetViewer, penv, viewername));
    penv->Load(scenefilename); // load the scene
    thviewer.join(); // wait for the viewer thread to exit
    penv->Destroy(); // destroy
    return 0;
}
