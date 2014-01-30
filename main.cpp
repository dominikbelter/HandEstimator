#include <iostream>
#include "handest_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Visualizer/visualizerGL.h"
#include "Visualizer/visualizerPCL.h"
#include "Filter/filterPCL.h"
#include "Core/Math/CMat44.h"
#include "Kinematic/kinematic_liego.h"
#include "OptimizationFunction/optimizationFunctionPF.h"
#include "Optimization/optimizationPSO.h"

#define _USE_MATH_DEFINES

using namespace std;

int main()
{
    try {
        using namespace handest;

        Point3D::Cloud cloud;
        Grabber* grabber = createGrabberKinect();
        grabber->LoadFromFile("../resources/Hand3.pcd");
        grabber->getCloud(cloud);

        Point3D::Cloud hand_cloud;

        Filter* filter = handest::createFilterPCL();
        filter->FilterScene(cloud, hand_cloud);


        //grabber->getCloud(cloud);  // converts the cloud to Point3D type
        Visualizer* visuPCL1 = createVisualizerPCL();
        RGBA color1;
        color1.r = 255;
        color1.g = 0;
        color1.b = 0;
        color1.a = 255;
        visuPCL1->addCloud(cloud, color1);
        visuPCL1->show();

        Visualizer* visuPCL2 = createVisualizerPCL();
        RGBA color2;
        color2.r = 0;
        color2.g = 255;
        color2.b = 0;
        color2.a = 255;
        visuPCL2->addCloud(hand_cloud, color2);
        visuPCL2->show();



        Hand::Pose hand;

        ForwardKinematics *fk = createForwardKinematicsLiego();
        //create optimization PSO
        handest::Optimization* optmPSO = createOptimizationPSO();
        // optimize with PSO
        optmPSO->Optimize(hand,hand_cloud);

        Visualizer* visuPCL3 = createVisualizerPCL();
        color2.r = 0;
        color2.g = 255;
        color2.b = 0;
        color2.a = 255;
        visuPCL3->addCloud(hand_cloud, color2);
        visuPCL3->show();



    }
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		return 1;
	}


	return 0;
}
