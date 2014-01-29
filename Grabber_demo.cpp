#include <iostream>
#include "handest_defs.h"
#include "Grabber/kinect_grabber.h"
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
		Grabber* grabber = createGrabberKinect();
		Point3D::Cloud scene;
		grabber->LoadFromFile("../resources/joints/palm.pcd");
		//grabber->run();   // grabs a single cloud from kinnect
		grabber->getCloud(scene);  // converts the cloud to Point3D type
		Visualizer* visuPCL = createVisualizerPCL();
		RGBA color;
		color.r = 255;
		color.g = 0;
		color.b = 0;
		color.a = 255;
		visuPCL->addCloud(scene, color);
		visuPCL->show();

}
