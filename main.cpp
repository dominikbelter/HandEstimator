#include <iostream>
#include "handest_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Visualizer/visualizerGL.h"
#include "Visualizer/visualizerPCL.h"
#include "Filter/filterPCL.h"
#include "Core/Math/CMat44.h"
#include "Kinematic/kinematic_liego.h"
#include "OptimizationFunction/optimizationFunctionPF.h"
#define _USE_MATH_DEFINES

using namespace std;

int main()
{
    try {
        using namespace handest;

        Grabber* grabber = createGrabberKinect();
	    //cout << "Current grabber: " << grabber->getName() << endl;
        Point3D::Cloud scene;
        cout << scene.size() << endl;
        //grabber->run();
        //cout << scene.size() << endl;
        //grabber->getCloud(scene);
        cout << scene.size() << endl;
        Visualizer* visualizer = createVisualizerGL();
        RGBA colour;
        colour.r = 1;
        colour.g = 0;
		colour.b = 0;
        colour.a = 255;

        //-----Vizualizer PCL test
        Visualizer* visuPCL = createVisualizerPCL();
        Point3D::Cloud chmura;
        RGBA color;
        color.r = 255;
        color.g = 0;
        color.b = 0;
        color.a = 255;

        chmura.resize(5);
        chmura[0].position.x=0.1;        chmura[0].position.y=0.1;        chmura[0].position.z=0.0;
        chmura[1].position.x=0.11;        chmura[1].position.y=0.11;        chmura[1].position.z=0.0;
        chmura[2].position.x=0.12;        chmura[2].position.y=0.12;        chmura[2].position.z=0.0;
        chmura[3].position.x=0.13;        chmura[3].position.y=0.13;        chmura[3].position.z=0.0;
        chmura[4].position.x=0.14;        chmura[4].position.y=0.14;        chmura[4].position.z=0.0;

        visuPCL->addCloud(chmura,color);

        chmura[0].position.x=0.15;        chmura[0].position.y=0.15;        chmura[0].position.z=0.0;
        chmura[1].position.x=0.16;        chmura[1].position.y=0.16;        chmura[1].position.z=0.0;
        chmura[2].position.x=0.17;        chmura[2].position.y=0.17;        chmura[2].position.z=0.0;
        chmura[3].position.x=0.18;        chmura[3].position.y=0.18;        chmura[3].position.z=0.0;
        chmura[4].position.x=0.19;        chmura[4].position.y=0.19;        chmura[4].position.z=0.0;

        color.r = 0;
        color.g = 255;

        visuPCL->addCloud(chmura,color);

        visuPCL->show();
        visuPCL->clear();
        //-----Vizualizer PCL test

        visualizer->addCloud(scene,colour);
        visualizer->show();
        Filter* filter = createFilterPCL();
        Point3D::Cloud hand_cloud;
        filter->FilterScene(scene, hand_cloud);

        Hand::Pose hand;
        //OptimizationFunction * optimization_function = createOptimizationGauss();
		//optimizationFunction * optimization_function = createOptimizationFunctionPF();
        //Optimization * optimization = createOptimizationPSO();
        //optimization->optimize(hand, hand_cloud);
        //optimization->save2File(hand);
        //visualizer->showHand(hand);

      //  CMat44 matrix1;
      //  matrix1.createTRMatrix(0, M_PI/2, 0, 0.1, 0.2, 0.3);
      ///  matrix1.showMatrix();
       // matrix1.inv(&matrix1);
      //  matrix1.showMatrix();
    }
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		return 1;
	}

    /*
	//OptmimizationFunctionPF test
	Hand::Pose dlon;
    Point3D::Cloud chmura;

    dlon.palm.surface.resize(1);
	dlon.fingers[0].chain[0].surface.resize(1);
	dlon.fingers[2].chain[0].surface.resize(2);
	dlon.fingers[3].chain[1].surface.resize(1);

    dlon.palm.surface[0].position.x=-3;
    dlon.palm.surface[0].position.y=0;
	dlon.palm.surface[0].position.z=-4;
    dlon.fingers[0].chain[0].surface[0].position.x=0;
    dlon.fingers[0].chain[0].surface[0].position.y=-1;
	dlon.fingers[0].chain[0].surface[0].position.z=1;
	dlon.fingers[2].chain[0].surface[0].position.x=1;
    dlon.fingers[2].chain[0].surface[0].position.y=1;
	dlon.fingers[2].chain[0].surface[0].position.z=2;
	dlon.fingers[2].chain[0].surface[1].position.x=0;
    dlon.fingers[2].chain[0].surface[1].position.y=3;
	dlon.fingers[2].chain[0].surface[1].position.z=-2;
	dlon.fingers[3].chain[1].surface[0].position.x=3;
    dlon.fingers[3].chain[1].surface[0].position.y=3;
	dlon.fingers[3].chain[1].surface[0].position.z=4;

    chmura.resize(5);
    chmura[0].position.x=-4;
    chmura[0].position.y=0;
	chmura[0].position.z=1;
    chmura[1].position.x=-1;
    chmura[1].position.y=2;
	chmura[1].position.z=0;
    chmura[2].position.x=3;
    chmura[2].position.y=4;
	chmura[2].position.z=-2;
	chmura[3].position.x=3;
    chmura[3].position.y=0;
	chmura[3].position.z=1;
    chmura[4].position.x=2;
    chmura[4].position.y=-2;
	chmura[4].position.z=-3;

	optimizationFunction * optimization_function = createOptimizationFunctionPF();

    handest::float_t fitness=optimization_function->FitnessValue(dlon,chmura);
    cout<<"Fitness: "<<fitness<<endl;*/

	// Before using, please fill:
	// -Mat34 pose of the hand
	// -Mat34 pose of the finger
	// -lengths of each finger's links
	ForwardKinematics *fk = new ForwardKinematicsLiego();

	//fk->forward(hand, config)

	return 0;
}
