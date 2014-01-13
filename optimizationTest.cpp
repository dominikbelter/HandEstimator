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
    dlon.palm.surface[0].position.x=-13;
    dlon.palm.surface[0].position.y=10;
	dlon.palm.surface[0].position.z=-14;
	dlon.palm.surface[1].position.x=-13;
    dlon.palm.surface[1].position.y=10;
	dlon.palm.surface[1].position.z=-14;

	//finger 0
    dlon.fingers[0].chain[0].surface[0].position.x=0.5;
    dlon.fingers[0].chain[0].surface[0].position.y=-1;
	dlon.fingers[0].chain[0].surface[0].position.z=1;
	dlon.fingers[0].chain[0].surface[1].position.x=10;
    dlon.fingers[0].chain[0].surface[1].position.y=-1;
	dlon.fingers[0].chain[0].surface[1].position.z=1;

	dlon.fingers[0].chain[1].surface[0].position.x=10;
    dlon.fingers[0].chain[1].surface[0].position.y=-1;
	dlon.fingers[0].chain[1].surface[0].position.z=13;
	dlon.fingers[0].chain[1].surface[1].position.x=0;
    dlon.fingers[0].chain[1].surface[1].position.y=-1;
	dlon.fingers[0].chain[1].surface[1].position.z=11;

	dlon.fingers[0].chain[2].surface[0].position.x=20;
    dlon.fingers[0].chain[2].surface[0].position.y=-1;
	dlon.fingers[0].chain[2].surface[0].position.z=15;
	dlon.fingers[0].chain[2].surface[1].position.x=0;
    dlon.fingers[0].chain[2].surface[1].position.y=-17;
	dlon.fingers[0].chain[2].surface[1].position.z=1;

	//finger 1
	dlon.fingers[1].chain[0].surface[0].position.x=10;
    dlon.fingers[1].chain[0].surface[0].position.y=-1;
	dlon.fingers[1].chain[0].surface[0].position.z=11;
	dlon.fingers[1].chain[0].surface[1].position.x=0;
    dlon.fingers[1].chain[0].surface[1].position.y=-11;
	dlon.fingers[1].chain[0].surface[1].position.z=1;

	dlon.fingers[1].chain[1].surface[0].position.x=10;
    dlon.fingers[1].chain[1].surface[0].position.y=-1;
	dlon.fingers[1].chain[1].surface[0].position.z=11;
	dlon.fingers[1].chain[1].surface[1].position.x=50;
    dlon.fingers[1].chain[1].surface[1].position.y=-1;
	dlon.fingers[1].chain[1].surface[1].position.z=11;

	dlon.fingers[1].chain[2].surface[0].position.x=10;
    dlon.fingers[1].chain[2].surface[0].position.y=-13;
	dlon.fingers[1].chain[2].surface[0].position.z=16;
	dlon.fingers[1].chain[2].surface[1].position.x=10;
    dlon.fingers[1].chain[2].surface[1].position.y=-19;
	dlon.fingers[1].chain[2].surface[1].position.z=1;

	//finger 2
	dlon.fingers[2].chain[0].surface[0].position.x=10;
    dlon.fingers[2].chain[0].surface[0].position.y=-18;
	dlon.fingers[2].chain[0].surface[0].position.z=1;
	dlon.fingers[2].chain[0].surface[1].position.x=0;
    dlon.fingers[2].chain[0].surface[1].position.y=-18;
	dlon.fingers[2].chain[0].surface[1].position.z=18;

	dlon.fingers[2].chain[1].surface[0].position.x=10;
    dlon.fingers[2].chain[1].surface[0].position.y=-13;
	dlon.fingers[2].chain[1].surface[0].position.z=1;
	dlon.fingers[2].chain[1].surface[1].position.x=0;
    dlon.fingers[2].chain[1].surface[1].position.y=-11;
	dlon.fingers[2].chain[1].surface[1].position.z=12;

	dlon.fingers[2].chain[2].surface[0].position.x=3;
    dlon.fingers[2].chain[2].surface[0].position.y=-6;
	dlon.fingers[2].chain[2].surface[0].position.z=5;
	dlon.fingers[2].chain[2].surface[1].position.x=0;
    dlon.fingers[2].chain[2].surface[1].position.y=-4;
	dlon.fingers[2].chain[2].surface[1].position.z=8;

	//finger 3
	dlon.fingers[3].chain[0].surface[0].position.x=1;
    dlon.fingers[3].chain[0].surface[0].position.y=-1;
	dlon.fingers[3].chain[0].surface[0].position.z=7;
	dlon.fingers[3].chain[0].surface[1].position.x=0;
    dlon.fingers[3].chain[0].surface[1].position.y=-1;
	dlon.fingers[3].chain[0].surface[1].position.z=6;

	dlon.fingers[3].chain[1].surface[0].position.x=6;
    dlon.fingers[3].chain[1].surface[0].position.y=-3;
	dlon.fingers[3].chain[1].surface[0].position.z=5;
	dlon.fingers[3].chain[1].surface[1].position.x=1;
    dlon.fingers[3].chain[1].surface[1].position.y=-1;
	dlon.fingers[3].chain[1].surface[1].position.z=111;

	dlon.fingers[3].chain[2].surface[0].position.x=10;
    dlon.fingers[3].chain[2].surface[0].position.y=-1;
	dlon.fingers[3].chain[2].surface[0].position.z=1;
	dlon.fingers[3].chain[2].surface[1].position.x=30;
    dlon.fingers[3].chain[2].surface[1].position.y=-1;
	dlon.fingers[3].chain[2].surface[1].position.z=31;

	//finger 4
	dlon.fingers[4].chain[0].surface[0].position.x=10;
    dlon.fingers[4].chain[0].surface[0].position.y=-1;
	dlon.fingers[4].chain[0].surface[0].position.z=41;
	dlon.fingers[4].chain[0].surface[1].position.x=0;
    dlon.fingers[4].chain[0].surface[1].position.y=-1;
	dlon.fingers[4].chain[0].surface[1].position.z=31;

	dlon.fingers[4].chain[1].surface[0].position.x=20;
    dlon.fingers[4].chain[1].surface[0].position.y=-1;
	dlon.fingers[4].chain[1].surface[0].position.z=31;
	dlon.fingers[4].chain[1].surface[1].position.x=50;
    dlon.fingers[4].chain[1].surface[1].position.y=-1;
	dlon.fingers[4].chain[1].surface[1].position.z=1;

	dlon.fingers[4].chain[2].surface[0].position.x=40;
    dlon.fingers[4].chain[2].surface[0].position.y=-1;
	dlon.fingers[4].chain[2].surface[0].position.z=1;
	dlon.fingers[4].chain[2].surface[1].position.x=0;
    dlon.fingers[4].chain[2].surface[1].position.y=-31;
	dlon.fingers[4].chain[2].surface[1].position.z=21;

	//points in cloud
    chmura.resize(32);
    chmura[0].position.x=-14;
    chmura[0].position.y=0;
	chmura[0].position.z=5;
    chmura[1].position.x=-11;
    chmura[1].position.y=12;
	chmura[1].position.z=10;
    chmura[2].position.x=31;
    chmura[2].position.y=42;
	chmura[2].position.z=-22;
	chmura[3].position.x=34;
    chmura[3].position.y=0;
	chmura[3].position.z=1.5;
    chmura[4].position.x=-32;
    chmura[4].position.y=-25;
	chmura[4].position.z=-3;
	chmura[5].position.x=-14;
    chmura[5].position.y=0;
	chmura[5].position.z=11;
    chmura[6].position.x=-1;
    chmura[6].position.y=23;
	chmura[6].position.z=20;
    chmura[7].position.x=23;
    chmura[7].position.y=43;
	chmura[7].position.z=-22;
	chmura[8].position.x=3;
    chmura[8].position.y=10;
	chmura[8].position.z=19;
    chmura[9].position.x=26;
    chmura[9].position.y=-2;
	chmura[9].position.z=-13;
	chmura[10].position.x=-4;
    chmura[10].position.y=70;
	chmura[10].position.z=1;
    chmura[11].position.x=-17;
    chmura[11].position.y=23;
	chmura[11].position.z=0;
    chmura[12].position.x=3;
    chmura[12].position.y=44;
	chmura[12].position.z=-32;
	chmura[13].position.x=37;
    chmura[13].position.y=20;
	chmura[13].position.z=17;
    chmura[14].position.x=21;
    chmura[14].position.y=-2;
	chmura[14].position.z=-39;
	chmura[15].position.x=-4;
    chmura[15].position.y=0.7;
	chmura[15].position.z=11;
    chmura[16].position.x=-1;
    chmura[16].position.y=21;
	chmura[16].position.z=10;
    chmura[17].position.x=3;
    chmura[17].position.y=4;
	chmura[17].position.z=-21;
	chmura[18].position.x=3;
    chmura[18].position.y=7.9;
	chmura[18].position.z=1;
    chmura[19].position.x=12;
    chmura[19].position.y=-2;
	chmura[19].position.z=-33;
	chmura[20].position.x=-14;
    chmura[20].position.y=60;
	chmura[20].position.z=11;
    chmura[21].position.x=-11;
    chmura[21].position.y=62;
	chmura[21].position.z=20;
    chmura[22].position.x=3;
    chmura[22].position.y=48;
	chmura[22].position.z=-24;
	chmura[23].position.x=36;
    chmura[23].position.y=0;
	chmura[23].position.z=11;
    chmura[24].position.x=23;
    chmura[24].position.y=-22;
	chmura[24].position.z=-3;
	chmura[25].position.x=-54;
    chmura[25].position.y=20;
	chmura[25].position.z=11;
    chmura[26].position.x=-16;
    chmura[26].position.y=12;
	chmura[26].position.z=56;
    chmura[27].position.x=3;
    chmura[27].position.y=41;
	chmura[27].position.z=-42;
	chmura[28].position.x=31;
    chmura[28].position.y=14.5;
	chmura[28].position.z=10;
    chmura[29].position.x=22;
    chmura[29].position.y=-12;
	chmura[29].position.z=-310;
	chmura[30].position.x=30;
    chmura[30].position.y=0;
	chmura[30].position.z=11;
    chmura[31].position.x=2;
    chmura[31].position.y=-22;
	chmura[31].position.z=-3;

	handest::optimizationFunction * optimization_function = createOptimizationFunctionPF();

    handest::float_type fitness=optimization_function->FitnessValue(dlon,chmura);
    cout<<"Fitness: "<<fitness<<endl;
	//system("pause");

	handest::Optimization* optymalizacja = createOptimizationPSO();

	optymalizacja->Optimize(dlon,chmura);

	//fitness=optimization_function->FitnessValue(dlon,chmura);
    //cout<<"Fitness: "<<fitness<<endl;
	system("pause");

}