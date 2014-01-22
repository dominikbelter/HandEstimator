#include <iostream>
#include "handest_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Filter/filterPCL.h"
#include "Visualizer/visualizerPCL.h"
#include "Kinematic/kinematic_liego.h"
#include "../dependencies/Eigen/Eigen"
#include <string>
#include <fstream>

#ifndef WIN32
    #include <GL/glut.h>
#else
    #include <glut.h>
#endif

using namespace std;
using namespace handest;

int main()
{
     Point3D::Cloud cloud;
     Grabber* grabber = createGrabberKinect();
     grabber->LoadFromFile("../resources/Hand3.pcd");
     grabber->getCloud(cloud);

     Point3D::Cloud hand_cloud;

     Filter* filter = handest::createFilterPCL();
     filter->FilterScene(cloud, hand_cloud);

     //grabber->getCloud(cloud);  // converts the cloud to Point3D type
     Visualizer* visuPCL = createVisualizerPCL();
     RGBA color;
     color.r = 255;
     color.g = 0;
     color.b = 0;
     color.a = 255;
     visuPCL->addCloud(hand_cloud, color);
     visuPCL->show();
}
