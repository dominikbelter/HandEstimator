#include <iostream>
#include "handest_defs.h"
#include "OptimizationFunction/optimizationFunctionPF.h"
#include "Optimization/optimizationPSO.h"

#include "Grabber/kinect_grabber.h"
#include "Visualizer/visualizerPCL.h"
#include "Kinematic/kinematic_liego.h"
#include "../dependencies/Eigen/Eigen"
#include <string>
#include <fstream>

//#ifndef WIN32
//	#include <GL/glut.h>
//#else
//	#include <glut.h>
//#endif

using namespace std;
using namespace handest;

int main()
{
	enum finger {THUMB, INDEX, MIDDLE, RING, PINKY};
	//Optmimization test
	Hand::Pose hand;
    Point3D::Cloud chmura;

	//create optimizatiom points fitting function 
	handest::optimizationFunction * optimization_function = createOptimizationFunctionPF();

    //handest::float_type fitness=optimization_function->FitnessValue(dlon,chmura);
    //cout<<"Fitness: "<<fitness<<endl;
	//system("pause");

	//create optimization PSO
	handest::Optimization* optmPSO = createOptimizationPSO();
	// optimize with PSO
	optmPSO->Optimize(hand,chmura);

	cout<<"Final visualization"<<endl;
	Visualizer* visuPCL = createVisualizerPCL();
	// Change the size of each point (visualization parameter)
	((VisualizerPCL*) visuPCL)->setPointSize(10);

	// Adding clouds
	RGBA red(255, 0, 0);
	visuPCL->addCloud(hand.palm.surface, red);

	// Fingers got different colors for different parts to better visualize the results
	RGBA colors[5] = {RGBA(0,255,0), RGBA(0, 0, 255), RGBA(255,255,0), RGBA(255,0,255), RGBA(0,255,255)};
	for (int i=0;i<Hand::FINGERS; i++)
	{
		for (int j=0;j<3;j++)
		{
			visuPCL->addCloud(hand.fingers[THUMB + i].chain[j].surface, colors[(i+j)%5]);
		}
	}

	visuPCL->show();
	
	//fitness=optimization_function->FitnessValue(dlon,chmura);
    //cout<<"Fitness: "<<fitness<<endl;
    //cin.get();

}
