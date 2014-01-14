#include <iostream>
#include "handest_defs.h"
#include "Grabber/kinect_grabber.h"
#include "Visualizer/visualizerPCL.h"
#include "Kinematic/kinematic_liego.h"
#include "../dependencies/Eigen/Eigen"
#include <string>

#ifndef WIN32
	#include <GL/glut.h>
#else
	#include <glut.h>
#endif

using namespace std;
using namespace handest;

Mat34 eigen_2_mat34(const Eigen::Matrix4f &trans) {
	Mat34 pose;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			pose.R.m[i][j] = trans(i, j);
		}
		pose.p.v[i] = trans(i, 3);
	}
	return pose;
}

Eigen::Matrix4f mat34_2_eigen(const Mat34 &trans) {
	Eigen::Matrix4f pose;
	pose.setIdentity();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			pose(i, j) = trans.R.m[i][j];
		}
		pose(i, 3) = trans.p.v[i];
	}
	return pose;
}

Eigen::Vector4f vec3_2_eigen(Vec3 vec)
{
	Eigen::Vector4f res(vec.x, vec.y, vec.z, 1.0);
	return res;
}

Vec3 eigen_2_vec3(Eigen::Vector4f vec)
{
	Vec3 res;
	res.v[0] = vec(0);
	res.v[1] = vec(1);
	res.v[2] = vec(2);
	return res;
}


// Demo presenting the forward kinematics
int main()
{
	enum finger {THUMB, INDEX, MIDDLE, RING, PINKY};
	Hand::Pose hand;

    // Before using, please fill:
	// -lengths of each finger's links
	ForwardKinematics *fk = new ForwardKinematicsLiego();

	// Lengths of each link
	hand.fingers[THUMB].chain[0].length = 15.0;
	hand.fingers[THUMB].chain[1].length = 14.0;
	hand.fingers[THUMB].chain[2].length = 13.0;

	hand.fingers[INDEX].chain[0].length = 4.5/7;
	hand.fingers[INDEX].chain[1].length = 4.5/7;//;
	hand.fingers[INDEX].chain[2].length = 4.5/7;

	hand.fingers[MIDDLE].chain[0].length = 4.5/7;
	hand.fingers[MIDDLE].chain[1].length = 4.5/7;
	hand.fingers[MIDDLE].chain[2].length = 4.5/7;

	hand.fingers[RING].chain[0].length = 4.5/7;
	hand.fingers[RING].chain[1].length = 4.5/7;
	hand.fingers[RING].chain[2].length = 4.5/7;

	hand.fingers[PINKY].chain[0].length = 4.5/7;
	hand.fingers[PINKY].chain[1].length = 4.5/7;
	hand.fingers[PINKY].chain[2].length = 4.5/7;

	cout<<"Forward kinematic calculation" << endl;
	cout<<hand.palm.surface.size()<<endl;
	cout<<hand.fingers[0].chain[0].surface.size()<<endl;

	// Filled with zeros
	Hand::Config handConfig;
	handConfig.thumb[0] = 1.0;
	handConfig.thumb[1] = 0.0;
	handConfig.thumb[2] = 0.0;
	handConfig.thumb[3] = 0.0;
	handConfig.index[0] = 0.0;
	handConfig.index[1] = 0.0;
	handConfig.index[2] = 0.0;
	handConfig.index[3] = 1.0;
	handConfig.middle[0] = 1.0;
	handConfig.middle[1] = 0.0;
	handConfig.middle[2] = 0.0;
	handConfig.middle[3] = 0.0;
	handConfig.ring[0] = 0.0;
	handConfig.ring[1] = 0.0;
	handConfig.ring[2] = 0.0;
	handConfig.ring[3] = 0.0;
	handConfig.pinky[0] = 0.0;
	handConfig.pinky[1] = 0.0;
	handConfig.pinky[2] = 0.0;
	handConfig.pinky[3] = 0.0;
	fk->forward(hand, handConfig);
	delete fk;

	// Read clouds from file
	Grabber* grabber = createGrabberKinect();

	// Palm
	grabber->LoadFromFile("../../resources/joints/palm.pcd");
	grabber->getCloud(hand.palm.surface);
	// Cloud scaling
	for (int j = 0; j < hand.palm.surface.size(); j++) {
		for (int y = 0; y < 3; y++)
			hand.palm.surface[j].position.v[y] *= 20.0;
	}

	// Fingers
	string fingerCloudNames[3] = { "../../resources/joints/finger_bottom.pcd",
			"../../resources/joints/finger_bottom.pcd",
			"../../resources/joints/finger_bottom.pcd" };
	for (int i = 0; i < 3; i++) {
		grabber->LoadFromFile(fingerCloudNames[i]);
		for (int k = 0; k < 5; k++) {
			grabber->getCloud(hand.fingers[THUMB + k].chain[i].surface);

			// Cloud scaling
			for (int j=0;j<hand.fingers[THUMB + k].chain[i].surface.size();j++) {
				for (int y=0;y<3;y++)
					hand.fingers[THUMB + k].chain[i].surface[j].position.v[y] *= 20.0;
			}
		}
	}
	cout << "Ended clouds loading" << endl;


	// Mat34 pose of the hand
	Mat34 hand2cs;
	hand2cs.R.m[0][0] = hand2cs.R.m[1][1] = hand2cs.R.m[2][2] = 1.0;
	hand.pose = hand2cs;

	// Mat34 pose of the finger
	Mat34 finger2hand;
	finger2hand.R.m[0][0] = finger2hand.R.m[1][1] = finger2hand.R.m[2][2] = 1.0;
	finger2hand.p.v[0] = -1.3;
	finger2hand.p.v[1] = 0.0;
	finger2hand.p.v[2] = -0.2;
	hand.fingers[THUMB].pose = finger2hand;
	finger2hand.p.v[0] = -0.5;
	finger2hand.p.v[1] = 2.0;
	finger2hand.p.v[2] = -0.2;
	hand.fingers[INDEX].pose = finger2hand;
	finger2hand.p.v[0] = 0;
	finger2hand.p.v[1] = 2.0;
	finger2hand.p.v[2] = -0.2;
	hand.fingers[MIDDLE].pose = finger2hand;
	finger2hand.p.v[0] = 0.5;
	finger2hand.p.v[1] = 2.0;
	finger2hand.p.v[2] = -0.2;
	hand.fingers[RING].pose = finger2hand;
	finger2hand.p.v[0] = 1.0;
	finger2hand.p.v[1] = 2.0;
	finger2hand.p.v[2] = -0.2;
	hand.fingers[PINKY].pose = finger2hand;


	cout<<"Final visualization"<<endl;
    //
    // Final visualization
    //
	Visualizer* visuPCL = createVisualizerPCL();
	((VisualizerPCL*) visuPCL)->setPointSize(10);
	RGBA red(255, 0, 0), green(0,255,0), blue(0, 0, 255);

	Eigen::Matrix4f fk2vis = Eigen::Matrix4f::Zero();
	fk2vis(2,0) = fk2vis(0,1) = fk2vis(1,2) = fk2vis(3,3) = 1.0;

	for (int j=0;j<Hand::FINGERS;j++)
	{
		Eigen::Matrix4f trans = mat34_2_eigen(hand.fingers[j].pose);
		for (int k=0;k<hand.fingers[j].chain[0].surface.size();k++)
		{
			Eigen::Vector4f vec = vec3_2_eigen(hand.fingers[j].chain[0].surface[k].position);
			vec = trans * vec;
			hand.fingers[j].chain[0].surface[k].position = eigen_2_vec3(vec);
		}
		Eigen::Matrix4f next = mat34_2_eigen(hand.fingers[j].chain[1].pose);
		next = fk2vis * next * fk2vis.inverse();

		for (int k=0;k<hand.fingers[j].chain[1].surface.size();k++)
		{
			Eigen::Vector4f vec = vec3_2_eigen(hand.fingers[j].chain[1].surface[k].position);
			vec = trans * next * vec;
			hand.fingers[j].chain[1].surface[k].position = eigen_2_vec3(vec);
		}

		Eigen::Matrix4f zzz = mat34_2_eigen(hand.fingers[j].chain[2].pose);
		zzz = fk2vis * zzz * fk2vis.inverse();
		for (int k = 0; k < hand.fingers[j].chain[2].surface.size(); k++) {
			Eigen::Vector4f vec = vec3_2_eigen(
					hand.fingers[j].chain[2].surface[k].position);
			vec = trans * next * zzz * vec;
			hand.fingers[j].chain[2].surface[k].position = eigen_2_vec3(vec);
		}
	}

	visuPCL->addCloud(hand.palm.surface, red);
    visuPCL->addCloud(hand.fingers[0].chain[0].surface, blue);

  //  visuPCL->addCloud(hand.fingers[0].chain[2].surface, red);

    visuPCL->addCloud(hand.fingers[1].chain[0].surface, green);
    visuPCL->addCloud(hand.fingers[1].chain[1].surface, blue);
    visuPCL->addCloud(hand.fingers[1].chain[2].surface, green);

    visuPCL->addCloud(hand.fingers[2].chain[0].surface, blue);
    visuPCL->addCloud(hand.fingers[2].chain[1].surface, green);
    visuPCL->addCloud(hand.fingers[2].chain[2].surface, blue);

    visuPCL->addCloud(hand.fingers[3].chain[0].surface, green);
    visuPCL->addCloud(hand.fingers[3].chain[1].surface, blue);
    visuPCL->addCloud(hand.fingers[3].chain[2].surface, green);

    visuPCL->addCloud(hand.fingers[4].chain[0].surface, blue);
    visuPCL->addCloud(hand.fingers[4].chain[1].surface, green);
    visuPCL->addCloud(hand.fingers[4].chain[2].surface, blue);


	visuPCL->show();

	return 0;
}
