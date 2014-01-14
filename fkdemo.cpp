#include <iostream>
#include "handest_defs.h"

#ifndef WIN32
	#include <GL/glut.h>
#else
	#include <glut.h>
#endif

using namespace std;
using namespace handest;


// Demo presenting the forward kinematics
int main()
{
	Hand::Pose hand;
    Point3D::Cloud visualization_cloud;

    // Defining the sizes of the hand
    hand.palm.surface.resize(2);
	hand.fingers[0].chain[0].surface.resize(2);
	hand.fingers[0].chain[1].surface.resize(2);
	hand.fingers[0].chain[2].surface.resize(2);
	hand.fingers[1].chain[0].surface.resize(2);
	hand.fingers[1].chain[1].surface.resize(2);
	hand.fingers[1].chain[2].surface.resize(2);
	hand.fingers[2].chain[0].surface.resize(2);
	hand.fingers[2].chain[1].surface.resize(2);
	hand.fingers[2].chain[2].surface.resize(2);
	hand.fingers[3].chain[0].surface.resize(2);
	hand.fingers[3].chain[1].surface.resize(2);
	hand.fingers[3].chain[2].surface.resize(2);
	hand.fingers[4].chain[0].surface.resize(2);
	hand.fingers[4].chain[1].surface.resize(2);
	hand.fingers[4].chain[2].surface.resize(2);

	//palm
    hand.palm.surface[0].position.x=-13;
    hand.palm.surface[0].position.y=10;
	hand.palm.surface[0].position.z=-14;
	hand.palm.surface[1].position.x=-13;
    hand.palm.surface[1].position.y=10;
	hand.palm.surface[1].position.z=-14;

	//finger 0
    hand.fingers[0].chain[0].surface[0].position.x=0.5;
    hand.fingers[0].chain[0].surface[0].position.y=-1;
	hand.fingers[0].chain[0].surface[0].position.z=1;
	hand.fingers[0].chain[0].surface[1].position.x=10;
    hand.fingers[0].chain[0].surface[1].position.y=-1;
	hand.fingers[0].chain[0].surface[1].position.z=1;

	hand.fingers[0].chain[1].surface[0].position.x=10;
    hand.fingers[0].chain[1].surface[0].position.y=-1;
	hand.fingers[0].chain[1].surface[0].position.z=13;
	hand.fingers[0].chain[1].surface[1].position.x=0;
    hand.fingers[0].chain[1].surface[1].position.y=-1;
	hand.fingers[0].chain[1].surface[1].position.z=11;

	hand.fingers[0].chain[2].surface[0].position.x=20;
    hand.fingers[0].chain[2].surface[0].position.y=-1;
	hand.fingers[0].chain[2].surface[0].position.z=15;
	hand.fingers[0].chain[2].surface[1].position.x=0;
    hand.fingers[0].chain[2].surface[1].position.y=-17;
	hand.fingers[0].chain[2].surface[1].position.z=1;

	//finger 1
	hand.fingers[1].chain[0].surface[0].position.x=10;
    hand.fingers[1].chain[0].surface[0].position.y=-1;
	hand.fingers[1].chain[0].surface[0].position.z=11;
	hand.fingers[1].chain[0].surface[1].position.x=0;
    hand.fingers[1].chain[0].surface[1].position.y=-11;
	hand.fingers[1].chain[0].surface[1].position.z=1;

	hand.fingers[1].chain[1].surface[0].position.x=10;
    hand.fingers[1].chain[1].surface[0].position.y=-1;
	hand.fingers[1].chain[1].surface[0].position.z=11;
	hand.fingers[1].chain[1].surface[1].position.x=50;
    hand.fingers[1].chain[1].surface[1].position.y=-1;
	hand.fingers[1].chain[1].surface[1].position.z=11;

	hand.fingers[1].chain[2].surface[0].position.x=10;
    hand.fingers[1].chain[2].surface[0].position.y=-13;
	hand.fingers[1].chain[2].surface[0].position.z=16;
	hand.fingers[1].chain[2].surface[1].position.x=10;
    hand.fingers[1].chain[2].surface[1].position.y=-19;
	hand.fingers[1].chain[2].surface[1].position.z=1;

	//finger 2
	hand.fingers[2].chain[0].surface[0].position.x=10;
    hand.fingers[2].chain[0].surface[0].position.y=-18;
	hand.fingers[2].chain[0].surface[0].position.z=1;
	hand.fingers[2].chain[0].surface[1].position.x=0;
    hand.fingers[2].chain[0].surface[1].position.y=-18;
	hand.fingers[2].chain[0].surface[1].position.z=18;

	hand.fingers[2].chain[1].surface[0].position.x=10;
    hand.fingers[2].chain[1].surface[0].position.y=-13;
	hand.fingers[2].chain[1].surface[0].position.z=1;
	hand.fingers[2].chain[1].surface[1].position.x=0;
    hand.fingers[2].chain[1].surface[1].position.y=-11;
	hand.fingers[2].chain[1].surface[1].position.z=12;

	hand.fingers[2].chain[2].surface[0].position.x=3;
    hand.fingers[2].chain[2].surface[0].position.y=-6;
	hand.fingers[2].chain[2].surface[0].position.z=5;
	hand.fingers[2].chain[2].surface[1].position.x=0;
    hand.fingers[2].chain[2].surface[1].position.y=-4;
	hand.fingers[2].chain[2].surface[1].position.z=8;

	//finger 3
	hand.fingers[3].chain[0].surface[0].position.x=1;
    hand.fingers[3].chain[0].surface[0].position.y=-1;
	hand.fingers[3].chain[0].surface[0].position.z=7;
	hand.fingers[3].chain[0].surface[1].position.x=0;
    hand.fingers[3].chain[0].surface[1].position.y=-1;
	hand.fingers[3].chain[0].surface[1].position.z=6;

	hand.fingers[3].chain[1].surface[0].position.x=6;
    hand.fingers[3].chain[1].surface[0].position.y=-3;
	hand.fingers[3].chain[1].surface[0].position.z=5;
	hand.fingers[3].chain[1].surface[1].position.x=1;
    hand.fingers[3].chain[1].surface[1].position.y=-1;
	hand.fingers[3].chain[1].surface[1].position.z=111;

	hand.fingers[3].chain[2].surface[0].position.x=10;
    hand.fingers[3].chain[2].surface[0].position.y=-1;
	hand.fingers[3].chain[2].surface[0].position.z=1;
	hand.fingers[3].chain[2].surface[1].position.x=30;
    hand.fingers[3].chain[2].surface[1].position.y=-1;
	hand.fingers[3].chain[2].surface[1].position.z=31;

	//finger 4
	hand.fingers[4].chain[0].surface[0].position.x=10;
    hand.fingers[4].chain[0].surface[0].position.y=-1;
	hand.fingers[4].chain[0].surface[0].position.z=41;
	hand.fingers[4].chain[0].surface[1].position.x=0;
    hand.fingers[4].chain[0].surface[1].position.y=-1;
	hand.fingers[4].chain[0].surface[1].position.z=31;

	hand.fingers[4].chain[1].surface[0].position.x=20;
    hand.fingers[4].chain[1].surface[0].position.y=-1;
	hand.fingers[4].chain[1].surface[0].position.z=31;
	hand.fingers[4].chain[1].surface[1].position.x=50;
    hand.fingers[4].chain[1].surface[1].position.y=-1;
	hand.fingers[4].chain[1].surface[1].position.z=1;

	hand.fingers[4].chain[2].surface[0].position.x=40;
    hand.fingers[4].chain[2].surface[0].position.y=-1;
	hand.fingers[4].chain[2].surface[0].position.z=1;
	hand.fingers[4].chain[2].surface[1].position.x=0;
    hand.fingers[4].chain[2].surface[1].position.y=-31;
	hand.fingers[4].chain[2].surface[1].position.z=21;

	//points in cloud
    visualization_clouds.resize(32);
    visualization_clouds[0].position.x=-14;
    visualization_clouds[0].position.y=0;
	visualization_clouds[0].position.z=5;
    visualization_clouds[1].position.x=-11;
    visualization_clouds[1].position.y=12;
	visualization_clouds[1].position.z=10;
    visualization_clouds[2].position.x=31;
    visualization_clouds[2].position.y=42;
	visualization_clouds[2].position.z=-22;
	visualization_clouds[3].position.x=34;
    visualization_clouds[3].position.y=0;
	visualization_clouds[3].position.z=1.5;
    visualization_clouds[4].position.x=-32;
    visualization_clouds[4].position.y=-25;
	visualization_clouds[4].position.z=-3;
	visualization_clouds[5].position.x=-14;
    visualization_clouds[5].position.y=0;
	visualization_clouds[5].position.z=11;
    visualization_clouds[6].position.x=-1;
    visualization_clouds[6].position.y=23;
	visualization_clouds[6].position.z=20;
    visualization_clouds[7].position.x=23;
    visualization_clouds[7].position.y=43;
	visualization_clouds[7].position.z=-22;
	visualization_clouds[8].position.x=3;
    visualization_clouds[8].position.y=10;
	visualization_clouds[8].position.z=19;
    visualization_clouds[9].position.x=26;
    visualization_clouds[9].position.y=-2;
	visualization_clouds[9].position.z=-13;
	visualization_clouds[10].position.x=-4;
    visualization_clouds[10].position.y=70;
	visualization_clouds[10].position.z=1;
    visualization_clouds[11].position.x=-17;
    visualization_clouds[11].position.y=23;
	visualization_clouds[11].position.z=0;
    visualization_clouds[12].position.x=3;
    visualization_clouds[12].position.y=44;
	visualization_clouds[12].position.z=-32;
	visualization_clouds[13].position.x=37;
    visualization_clouds[13].position.y=20;
	visualization_clouds[13].position.z=17;
    visualization_clouds[14].position.x=21;
    visualization_clouds[14].position.y=-2;
	visualization_clouds[14].position.z=-39;
	visualization_clouds[15].position.x=-4;
    visualization_clouds[15].position.y=0.7;
	visualization_clouds[15].position.z=11;
    visualization_clouds[16].position.x=-1;
    visualization_clouds[16].position.y=21;
	visualization_clouds[16].position.z=10;
    visualization_clouds[17].position.x=3;
    visualization_clouds[17].position.y=4;
	visualization_clouds[17].position.z=-21;
	visualization_clouds[18].position.x=3;
    visualization_clouds[18].position.y=7.9;
	visualization_clouds[18].position.z=1;
    visualization_clouds[19].position.x=12;
    visualization_clouds[19].position.y=-2;
	visualization_clouds[19].position.z=-33;
	visualization_clouds[20].position.x=-14;
    visualization_clouds[20].position.y=60;
	visualization_clouds[20].position.z=11;
    visualization_clouds[21].position.x=-11;
    visualization_clouds[21].position.y=62;
	visualization_clouds[21].position.z=20;
    visualization_clouds[22].position.x=3;
    visualization_clouds[22].position.y=48;
	visualization_clouds[22].position.z=-24;
	visualization_clouds[23].position.x=36;
    visualization_clouds[23].position.y=0;
	visualization_clouds[23].position.z=11;
    visualization_clouds[24].position.x=23;
    visualization_clouds[24].position.y=-22;
	visualization_clouds[24].position.z=-3;
	visualization_clouds[25].position.x=-54;
    visualization_clouds[25].position.y=20;
	visualization_clouds[25].position.z=11;
    visualization_clouds[26].position.x=-16;
    visualization_clouds[26].position.y=12;
	visualization_clouds[26].position.z=56;
    visualization_clouds[27].position.x=3;
    visualization_clouds[27].position.y=41;
	visualization_clouds[27].position.z=-42;
	visualization_clouds[28].position.x=31;
    visualization_clouds[28].position.y=14.5;
	visualization_clouds[28].position.z=10;
    visualization_clouds[29].position.x=22;
    visualization_clouds[29].position.y=-12;
	visualization_clouds[29].position.z=-310;
	visualization_clouds[30].position.x=30;
    visualization_clouds[30].position.y=0;
	visualization_clouds[30].position.z=11;
    visualization_clouds[31].position.x=2;
    visualization_clouds[31].position.y=-22;
	visualization_clouds[31].position.z=-3;

	return 0;
}
