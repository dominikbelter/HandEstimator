#include <iostream>
#include <glut.h>
#include "handest_defs.h"
#include "OptimizationFunction/optimizationFunctionPF.h"
#include "Optimization/optimizationPSO.h"

using namespace std;
using namespace handest;

int main()
{
	//Optmimization test
	Hand::Pose dlon;
    Point3D::Cloud chmura;


	//points in hand
    dlon.palm.surface.resize(2);
	dlon.fingers[0].chain[0].surface.resize(2);
	dlon.fingers[0].chain[1].surface.resize(2);
	dlon.fingers[0].chain[2].surface.resize(2);
	dlon.fingers[1].chain[0].surface.resize(2);
	dlon.fingers[1].chain[1].surface.resize(2);
	dlon.fingers[1].chain[2].surface.resize(2);
	dlon.fingers[2].chain[0].surface.resize(2);
	dlon.fingers[2].chain[1].surface.resize(2);
	dlon.fingers[2].chain[2].surface.resize(2);
	dlon.fingers[3].chain[0].surface.resize(2);
	dlon.fingers[3].chain[1].surface.resize(2);
	dlon.fingers[3].chain[2].surface.resize(2);
	dlon.fingers[4].chain[0].surface.resize(2);
	dlon.fingers[4].chain[1].surface.resize(2);
	dlon.fingers[4].chain[2].surface.resize(2);

	//palm
    dlon.palm.surface[0].position.x=-3;
    dlon.palm.surface[0].position.y=0;
	dlon.palm.surface[0].position.z=-4;
	dlon.palm.surface[1].position.x=-3;
    dlon.palm.surface[1].position.y=0;
	dlon.palm.surface[1].position.z=-4;

	//finger 0
    dlon.fingers[0].chain[0].surface[0].position.x=0;
    dlon.fingers[0].chain[0].surface[0].position.y=-1;
	dlon.fingers[0].chain[0].surface[0].position.z=1;
	dlon.fingers[0].chain[0].surface[1].position.x=0;
    dlon.fingers[0].chain[0].surface[1].position.y=-1;
	dlon.fingers[0].chain[0].surface[1].position.z=1;

	dlon.fingers[0].chain[1].surface[0].position.x=0;
    dlon.fingers[0].chain[1].surface[0].position.y=-1;
	dlon.fingers[0].chain[1].surface[0].position.z=1;
	dlon.fingers[0].chain[1].surface[1].position.x=0;
    dlon.fingers[0].chain[1].surface[1].position.y=-1;
	dlon.fingers[0].chain[1].surface[1].position.z=1;

	dlon.fingers[0].chain[2].surface[0].position.x=0;
    dlon.fingers[0].chain[2].surface[0].position.y=-1;
	dlon.fingers[0].chain[2].surface[0].position.z=1;
	dlon.fingers[0].chain[2].surface[1].position.x=0;
    dlon.fingers[0].chain[2].surface[1].position.y=-1;
	dlon.fingers[0].chain[2].surface[1].position.z=1;

	//finger 1
	dlon.fingers[1].chain[0].surface[0].position.x=0;
    dlon.fingers[1].chain[0].surface[0].position.y=-1;
	dlon.fingers[1].chain[0].surface[0].position.z=1;
	dlon.fingers[1].chain[0].surface[1].position.x=0;
    dlon.fingers[1].chain[0].surface[1].position.y=-1;
	dlon.fingers[1].chain[0].surface[1].position.z=1;

	dlon.fingers[1].chain[1].surface[0].position.x=0;
    dlon.fingers[1].chain[1].surface[0].position.y=-1;
	dlon.fingers[1].chain[1].surface[0].position.z=1;
	dlon.fingers[1].chain[1].surface[1].position.x=0;
    dlon.fingers[1].chain[1].surface[1].position.y=-1;
	dlon.fingers[1].chain[1].surface[1].position.z=1;

	dlon.fingers[1].chain[2].surface[0].position.x=0;
    dlon.fingers[1].chain[2].surface[0].position.y=-1;
	dlon.fingers[1].chain[2].surface[0].position.z=1;
	dlon.fingers[1].chain[2].surface[1].position.x=0;
    dlon.fingers[1].chain[2].surface[1].position.y=-1;
	dlon.fingers[1].chain[2].surface[1].position.z=1;

	//finger 2
	dlon.fingers[2].chain[0].surface[0].position.x=0;
    dlon.fingers[2].chain[0].surface[0].position.y=-1;
	dlon.fingers[2].chain[0].surface[0].position.z=1;
	dlon.fingers[2].chain[0].surface[1].position.x=0;
    dlon.fingers[2].chain[0].surface[1].position.y=-1;
	dlon.fingers[2].chain[0].surface[1].position.z=1;

	dlon.fingers[2].chain[1].surface[0].position.x=0;
    dlon.fingers[2].chain[1].surface[0].position.y=-1;
	dlon.fingers[2].chain[1].surface[0].position.z=1;
	dlon.fingers[2].chain[1].surface[1].position.x=0;
    dlon.fingers[2].chain[1].surface[1].position.y=-1;
	dlon.fingers[2].chain[1].surface[1].position.z=1;

	dlon.fingers[2].chain[2].surface[0].position.x=0;
    dlon.fingers[2].chain[2].surface[0].position.y=-1;
	dlon.fingers[2].chain[2].surface[0].position.z=1;
	dlon.fingers[2].chain[2].surface[1].position.x=0;
    dlon.fingers[2].chain[2].surface[1].position.y=-1;
	dlon.fingers[2].chain[2].surface[1].position.z=1;

	//finger 3
	dlon.fingers[3].chain[0].surface[0].position.x=0;
    dlon.fingers[3].chain[0].surface[0].position.y=-1;
	dlon.fingers[3].chain[0].surface[0].position.z=1;
	dlon.fingers[3].chain[0].surface[1].position.x=0;
    dlon.fingers[3].chain[0].surface[1].position.y=-1;
	dlon.fingers[3].chain[0].surface[1].position.z=1;

	dlon.fingers[3].chain[1].surface[0].position.x=0;
    dlon.fingers[3].chain[1].surface[0].position.y=-1;
	dlon.fingers[3].chain[1].surface[0].position.z=1;
	dlon.fingers[3].chain[1].surface[1].position.x=0;
    dlon.fingers[3].chain[1].surface[1].position.y=-1;
	dlon.fingers[3].chain[1].surface[1].position.z=1;

	dlon.fingers[3].chain[2].surface[0].position.x=0;
    dlon.fingers[3].chain[2].surface[0].position.y=-1;
	dlon.fingers[3].chain[2].surface[0].position.z=1;
	dlon.fingers[3].chain[2].surface[1].position.x=0;
    dlon.fingers[3].chain[2].surface[1].position.y=-1;
	dlon.fingers[3].chain[2].surface[1].position.z=1;

	//finger 4
	dlon.fingers[4].chain[0].surface[0].position.x=0;
    dlon.fingers[4].chain[0].surface[0].position.y=-1;
	dlon.fingers[4].chain[0].surface[0].position.z=1;
	dlon.fingers[4].chain[0].surface[1].position.x=0;
    dlon.fingers[4].chain[0].surface[1].position.y=-1;
	dlon.fingers[4].chain[0].surface[1].position.z=1;

	dlon.fingers[4].chain[1].surface[0].position.x=0;
    dlon.fingers[4].chain[1].surface[0].position.y=-1;
	dlon.fingers[4].chain[1].surface[0].position.z=1;
	dlon.fingers[4].chain[1].surface[1].position.x=0;
    dlon.fingers[4].chain[1].surface[1].position.y=-1;
	dlon.fingers[4].chain[1].surface[1].position.z=1;

	dlon.fingers[4].chain[2].surface[0].position.x=0;
    dlon.fingers[4].chain[2].surface[0].position.y=-1;
	dlon.fingers[4].chain[2].surface[0].position.z=1;
	dlon.fingers[4].chain[2].surface[1].position.x=0;
    dlon.fingers[4].chain[2].surface[1].position.y=-1;
	dlon.fingers[4].chain[2].surface[1].position.z=1;

	//points in cloud
    chmura.resize(32);
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
	chmura[5].position.x=-4;
    chmura[5].position.y=0;
	chmura[5].position.z=1;
    chmura[6].position.x=-1;
    chmura[6].position.y=2;
	chmura[6].position.z=0;
    chmura[7].position.x=3;
    chmura[7].position.y=4;
	chmura[7].position.z=-2;
	chmura[8].position.x=3;
    chmura[8].position.y=0;
	chmura[8].position.z=1;
    chmura[9].position.x=2;
    chmura[9].position.y=-2;
	chmura[9].position.z=-3;
	chmura[10].position.x=-4;
    chmura[10].position.y=0;
	chmura[10].position.z=1;
    chmura[11].position.x=-1;
    chmura[11].position.y=2;
	chmura[11].position.z=0;
    chmura[12].position.x=3;
    chmura[12].position.y=4;
	chmura[12].position.z=-2;
	chmura[13].position.x=3;
    chmura[13].position.y=0;
	chmura[13].position.z=1;
    chmura[14].position.x=2;
    chmura[14].position.y=-2;
	chmura[14].position.z=-3;
	chmura[15].position.x=-4;
    chmura[15].position.y=0;
	chmura[15].position.z=1;
    chmura[16].position.x=-1;
    chmura[16].position.y=2;
	chmura[16].position.z=0;
    chmura[17].position.x=3;
    chmura[17].position.y=4;
	chmura[17].position.z=-2;
	chmura[18].position.x=3;
    chmura[18].position.y=0;
	chmura[18].position.z=1;
    chmura[19].position.x=2;
    chmura[19].position.y=-2;
	chmura[19].position.z=-3;
	chmura[20].position.x=-4;
    chmura[20].position.y=0;
	chmura[20].position.z=1;
    chmura[21].position.x=-1;
    chmura[21].position.y=2;
	chmura[21].position.z=0;
    chmura[22].position.x=3;
    chmura[22].position.y=4;
	chmura[22].position.z=-2;
	chmura[23].position.x=3;
    chmura[23].position.y=0;
	chmura[23].position.z=1;
    chmura[24].position.x=2;
    chmura[24].position.y=-2;
	chmura[24].position.z=-3;
	/*chmura[25].position.x=-4;
    chmura[25].position.y=0;
	chmura[25].position.z=1;
    chmura[26].position.x=-1;
    chmura[26].position.y=2;
	chmura[26].position.z=0;
    chmura[27].position.x=3;
    chmura[27].position.y=4;
	chmura[27].position.z=-2;
	chmura[28].position.x=3;
    chmura[28].position.y=0;
	chmura[28].position.z=1;
    chmura[29].position.x=2;
    chmura[29].position.y=-2;
	chmura[29].position.z=-3;
	chmura[30].position.x=3;
    chmura[30].position.y=0;
	chmura[30].position.z=1;
    chmura[31].position.x=2;
    chmura[31].position.y=-2;
	chmura[31].position.z=-3;*/

	handest::optimizationFunction * optimization_function = createOptimizationFunctionPF();

    //handest::float_type fitness=optimization_function->FitnessValue(dlon,chmura);
    //cout<<"Fitness: "<<fitness<<endl;
	//system("pause");

	//handest::Optimization* optymalizacja = createOptimizationPSO();

	//optimization->Optimize(dlon,chmura);



}