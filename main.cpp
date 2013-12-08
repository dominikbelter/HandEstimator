#include <iostream>
#include "handest_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Visualizer/visualizerGL.h"
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
        cout << "Current grabber: " << grabber->getName() << endl;
        Point3D::Cloud scene;
        grabber->grab();
        grabber->getCloud(scene);
        Visualizer* visualizer = createVisualizerGL();
		RGBA colour;
		colour.r = 1;
		colour.g = 0;
		colour.b = 0;
		colour.a = 1;
		visualizer->addCloud(scene,colour);
        visualizer->show();
        Filter* filter = createFilterPCL();
        Point3D::Cloud hand_cloud;
        filter->FilterScene(scene, hand_cloud);

        Hand::Pose hand;
        //OptimizationFunction * optimization_function = createOptimizationGauss();
		//optimizationFunction * optimization_function = createOptimizationFunctionPF();
        //Optimization * optimization = createOptimizationPSO();
        //optimization->optimize(hand_cloud, hand);
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
