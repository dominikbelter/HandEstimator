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

Eigen::Vector4f vec3_2_eigen(Vec3 vec) {
	Eigen::Vector4f res(vec.x, vec.y, vec.z, 1.0);
	return res;
}

Vec3 eigen_2_vec3(Eigen::Vector4f vec) {
	Vec3 res;
	res.v[0] = vec(0);
	res.v[1] = vec(1);
	res.v[2] = vec(2);
	return res;
}

/// Demo of the forward kinematic using kinematic Liego
int main() {
	enum finger {
		THUMB, INDEX, MIDDLE, RING, PINKY
	};
	Hand::Pose hand;
	Hand::Config handConfig;

	ForwardKinematics *fk = createForwardKinematicsLiego();

	// Reading the config file with the radian angles of the joints
	ifstream ostream;
	ostream.open("config.cfg");
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 4; j++) {
			string a;
			ostream >> a;
			handConfig.conf[i * 4 + j] = atof(a.c_str());
		}
		cout << endl;
	}
	ostream.close();

	// Lengths of joints - might need to customize as different finger have different lengths
	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 3; j++)
			hand.fingers[THUMB + i].chain[j].length = 4.5 / 7;

	// Computing forward kinematic
	fk->forward(hand, handConfig);

	//
	// Read clouds from file
	//
	Grabber* grabber = createGrabberKinect();

	// Palm
	grabber->LoadFromFile("../../resources/joints/palm.pcd");
	grabber->getCloud(hand.palm.surface);
	for (int j = 0; j < hand.palm.surface.size(); j++) {
		for (int y = 0; y < 3; y++)
			hand.palm.surface[j].position.v[y] *= 20.0;
	}

	// Fingers
	string fingerCloudNames[3] = { "../../resources/joints2/finger_bottom.pcd",
			"../../resources/joints2/finger_middle.pcd",
			"../../resources/joints2/finger_top.pcd" };
	string thumbCloudNames[3] = { "../../resources/joints2/thumb_bottom.pcd",
			"../../resources/joints2/thumb_middle.pcd",
			"../../resources/joints2/thumb_top.pcd" };

	for (int k = 0; k < 5; k++) {
		for (int i = 0; i < 3; i++) {
			if (k == 0)
				grabber->LoadFromFile(fingerCloudNames[i]);
			else
				grabber->LoadFromFile(thumbCloudNames[i]);

			grabber->getCloud(hand.fingers[THUMB + k].chain[i].surface);

			// Cloud scaling for better visualization
			for (int j = 0; j < hand.fingers[THUMB + k].chain[i].surface.size();
					j++) {
				for (int y = 0; y < 3; y++)
					hand.fingers[THUMB + k].chain[i].surface[j].position.v[y] *=
							20.0;
			}
		}
	}
	cout << "All clouds loaded successfully" << endl;

	// Pose of the hand in the global coordinate system
	Mat34 hand2cs;
	hand2cs.R.m[0][0] = hand2cs.R.m[1][1] = hand2cs.R.m[2][2] = 1.0;
	hand.pose = hand2cs;

	// Mounting places of the fingers in the hand coordinate system
	Mat34 finger2hand;
	finger2hand.R.m[0][0] = finger2hand.R.m[1][1] = finger2hand.R.m[2][2] = 1.0;
	finger2hand.p.v[0] = -1.3;
	finger2hand.p.v[1] = 0.5;
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

	//
	// Transforming clouds
	//
	cout << "Transforming clouds according to computed transformation" << endl;

	// Transforming palm
	for (int k = 0; k < hand.palm.surface.size(); k++) {
		Eigen::Vector4f vec = vec3_2_eigen(hand.palm.surface[k].position);
		vec = mat34_2_eigen(hand.pose) * vec;
		hand.palm.surface[k].position = eigen_2_vec3(vec);
	}

	// Conversion between coordinate system of clouds representing part of finger and
	// coordinate system used in forward kinematic
	Eigen::Matrix4f fk2vis = Eigen::Matrix4f::Zero();
	fk2vis(2, 0) = fk2vis(0, 1) = fk2vis(1, 2) = fk2vis(3, 3) = 1.0;

	// Recalculation of already put clouds accordingly to the calculated fk
	for (int j = 0; j < Hand::FINGERS; j++) {
		// Relative transformations to the absolute ones
		Eigen::Matrix4f trans[4];
		trans[0] = mat34_2_eigen(hand.pose)
				* mat34_2_eigen(hand.fingers[j].pose);

		trans[1] = mat34_2_eigen(hand.fingers[j].chain[0].poseEnd);
		trans[1] = fk2vis * trans[1] * fk2vis.inverse();
		trans[1] = trans[0] * trans[1];

		trans[2] = mat34_2_eigen(hand.fingers[j].chain[1].poseEnd);
		trans[2] = fk2vis * trans[2] * fk2vis.inverse();
		trans[2] = trans[1] * trans[2];

		trans[3] = mat34_2_eigen(hand.fingers[j].chain[2].poseEnd);
		trans[3] = fk2vis * trans[3] * fk2vis.inverse();
		trans[3] = trans[2] * trans[3];

		// For all fingers, move the clouds
		for (int i = 0; i < 3; i++) {
			for (int k = 0; k < hand.fingers[j].chain[i].surface.size(); k++) {
				Eigen::Vector4f vec = vec3_2_eigen(
						hand.fingers[j].chain[i].surface[k].position);
				vec(1) -= hand.fingers[j].chain[i].length;
				vec = trans[i + 1] * vec;
				hand.fingers[j].chain[i].surface[k].position = eigen_2_vec3(
						vec);
			}
		}
	}

	//
	// Final visualization
	//
	std::cout << std::endl << " Final visualization " << std::endl;


	Visualizer* visuPCL = createVisualizerPCL();
	((VisualizerPCL*) visuPCL)->setPointSize(10);
	RGBA red(255, 0, 0);

	// Adding palm clouds
	visuPCL->addCloud(hand.palm.surface, red);

	// Fingers got different colors for different parts to better visualize the results
	RGBA colors[5] = { RGBA(0, 255, 0), RGBA(0, 0, 255), RGBA(255, 255, 0),
			RGBA(255, 0, 255), RGBA(0, 255, 255) };
	for (int i = 0; i < Hand::FINGERS; i++) {
		for (int j = 0; j < 3; j++) {
			visuPCL->addCloud(hand.fingers[THUMB + i].chain[j].surface,
					colors[(i + j) % 5]);
		}
	}

	// Show all!
	visuPCL->show();
	visuPCL->clear();

	// Go home ... :)
	return 0;
}
