#include <iostream>
#include "handest_defs.h"
#include "Grabber/kinect_grabber.h"
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
        //Visualizer* visualizer = createVisualizerGL();
        //visualizer->showCloud(scene);
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

/*    Hand::Pose dlon;
    Point3D::Cloud chmura;

    dlon.palm.surface.resize(3);
    dlon.palm.surface[0].position.x=0;
    dlon.palm.surface[0].position.y=0;
    dlon.palm.surface[1].position.x=-5;
    dlon.palm.surface[1].position.y=-5;
    dlon.palm.surface[2].position.x=5;
    dlon.palm.surface[2].position.y=5;

    chmura.resize(3);
    chmura[0].position.x=5;
    chmura[0].position.y=5;
    chmura[1].position.x=0;
    chmura[1].position.y=0;
    chmura[2].position.x=10;
    chmura[2].position.y=10;


	optimizationFunction * optimization_function = createOptimizationFunctionPF();

    handest::float_t fitness=optimization_function->FitnessValue(dlon,chmura);
    cout<<"Fitness: "<<fitness<<endl;
*/

	// Before using, please fill:
	// -Mat34 pose of the hand
	// -Mat34 pose of the finger
	// -lengths of each finger's links
	ForwardKinematics *fk = new ForwardKinematicsLiego();

	//fk->forward(hand, config)

	return 0;
}
