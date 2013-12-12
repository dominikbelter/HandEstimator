#include "../include/OptimizationFunction/optimizationFunctionGauss.h"

using namespace handest;

void optimizationFunctionGauss::getPointsFromHand(Hand::Pose& hand)
{	
	//std::cout<<"Palm points: "<<hand.palm.surface.size()<<"\n";
	for(int i=0;i<hand.palm.surface.size();i++)
		handPoints.push_back(hand.palm.surface[0]);

	//std::cout<<"Fingers: "<<int(Hand::FINGERS)<<"\n";
	for(int i=0;i<Hand::FINGERS;i++)
	{
		//std::cout<<"\tFinger "<<i<<". Links: "<<int(Finger::LINKS)<<"\n";
		for(int j=0;j<Finger::LINKS;j++)
		{
			//std::cout<<"\t\tLink "<<j<<". points: "<<hand.fingers[i].chain[j].surface.size()<<"\n";
			for(int k=0;k<hand.fingers[i].chain[j].surface.size();k++)
			handPoints.push_back(hand.fingers[i].chain[j].surface[k]);
		}
	}

	//std::cout<<"Hand Points Count: "<<handPoints.size()<<"\n";
}


floatGauss optimizationFunctionGauss::gaussFunction(Matrix<floatGauss,3,1> points,Point3D::Cloud& cloud)
{
	floatGauss overlap=0,handValue,cloudValue;
	Matrix<floatGauss,3,1> cloudPointsXYZ;
	Matrix<floatGauss,3,1> handPointsXYZ;

	for(int i=0;i<cloud.size();i++)
	{
		cloudPointsXYZ(0)=cloud[i].position.x;
		cloudPointsXYZ(1)=cloud[i].position.y;
		cloudPointsXYZ(2)=cloud[i].position.z;

		cloudValue=(1/(sqrt(pow(2*M_PI,3)*covarianceMatrix.determinant())))*exp(-.5*(points-cloudPointsXYZ).transpose()*covarianceMatrix.inverse()*(points-cloudPointsXYZ));

		for(int j=0;j<handPoints.size();j++)
			{
			handPointsXYZ(0)=handPoints[j].position.x;
			handPointsXYZ(1)=handPoints[j].position.y;
			handPointsXYZ(2)=handPoints[j].position.z;

			handValue=(1/(sqrt(pow(2*M_PI,3)*covarianceMatrix.determinant())))*exp(-.5*(points-handPointsXYZ).transpose()*covarianceMatrix.inverse()*(points-handPointsXYZ));
			overlap=handValue*cloudValue+overlap;
			}
	}
	return overlap;
}

floatGauss optimizationFunctionGauss::FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud)
{
	floatGauss summaryOverlap=0;

	/// covariance values
	covariance_x=0.1;
	covariance_y=0.1;
	covariance_z=0.1;

	/// maximum sampled cloud values
	nX=640;
	nY=480;
	nZ=1024;

	/// making covariance matrix
	covarianceMatrix(0,0)=covariance_x;
	covarianceMatrix(1,1)=covariance_y;
	covarianceMatrix(2,2)=covariance_z;

	Matrix<floatGauss,3,1>points;
	
	/// summing overlap of gaussian functions for hand and cloud
	for (int sampledX=0;nX;sampledX++)
		for (int sampledY=0;nY;sampledY++)
			for (int sampledZ=0;nZ;sampledZ++)
			{
				points(0)=sampledX;
				points(1)=sampledY;
				points(2)=sampledZ;

				summaryOverlap=summaryOverlap+gaussFunction(points,cloud);
			}


	return summaryOverlap;
}
