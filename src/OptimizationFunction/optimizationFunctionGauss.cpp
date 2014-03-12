#include "../include/OptimizationFunction/optimizationFunctionGauss.h"

using namespace handest;

/// A single instance of Point Fitting Optimization Function
optimizationFunctionGauss::Ptr optFuncGauss;

floatGauss optimizationFunctionGauss::FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud)
{
    handPoints.clear();
	getPointsFromHand(hand);

    floatGauss summaryOverlap=0;

    /// covariance values
    covariance_x=0.1;
    covariance_y=0.1;
    covariance_z=0.1;

    /// maximum sampled cloud values
    nX=1;
    nY=1;
    nZ=1;

    /// making covariance matrix
    covarianceMatrix(0,0)=covariance_x;
    covarianceMatrix(1,1)=covariance_y;
    covarianceMatrix(2,2)=covariance_z;

    Matrix<floatGauss,3,1>point;

    /// summing overlap of gaussian functions for hand and cloud
    for (int sampledX=0; sampledX<100; sampledX++)
        for (int sampledY=0; sampledY<100; sampledY++)
            for (int sampledZ=0; sampledZ<100; sampledZ++)
            {
                point(0)=(floatGauss)sampledX/100;
                point(1)=(floatGauss)sampledY/100;
                point(2)=(floatGauss)sampledZ/100;
     //           std::cout<<"x "<<point(0)<<",y "<<point(1)<<",z "<<point(2)<<std::endl;

                summaryOverlap=summaryOverlap+gaussFunction(point,cloud);
                std::cout<<"overlap "<<summaryOverlap<<std::endl;
            }

    return summaryOverlap;
}

void optimizationFunctionGauss::getPointsFromHand(Hand::Pose& hand)
{
    //std::cout<<"Palm points: "<<hand.palm.surface.size()<<"\n";
	for(size_t i=0;i<hand.palm.surface.size();i++)
		handPoints.push_back(hand.palm.surface[i]);

	for(size_t i=0;i<Hand::FINGERS;i++)
	{
		for(int j=0;j<Finger::LINKS;j++)
		{
			for(size_t k=0;k<hand.fingers[i].chain[j].surface.size();k++)
				handPoints.push_back(hand.fingers[i].chain[j].surface[k]);
		}
	}

  //  std::cout<<"Hand Points Count: "<<handPoints.size()<<"\n";
}


floatGauss optimizationFunctionGauss::gaussFunction(Matrix<floatGauss,3,1> point,Point3D::Cloud& cloud)
{
    floatGauss overlap=0,handValue=0,cloudValue=0;
    Matrix<floatGauss,3,1> cloudPointXYZ;
    Matrix<floatGauss,3,1> handPointXYZ;

    for(int i=0; i<cloud.size(); i++)
    {
        cloudPointXYZ(0)=cloud[i].position.x;
        cloudPointXYZ(1)=cloud[i].position.y;
        cloudPointXYZ(2)=cloud[i].position.z;

        cloudValue+=(1/(sqrt(pow(2*M_PI,3)*covarianceMatrix.determinant())))*exp(-.5*(point-cloudPointXYZ).transpose()*covarianceMatrix.inverse()*(point-cloudPointXYZ));
    }

    for(int j=0; j<handPoints.size(); j++)
    {
        handPointXYZ(0)=handPoints[j].position.x;
        handPointXYZ(1)=handPoints[j].position.y;
        handPointXYZ(2)=handPoints[j].position.z;
       // std::cout<<"hand "<<handPointXYZ(0)<<std::endl;

        handValue+=(1/(sqrt(pow(2*M_PI,3)*covarianceMatrix.determinant())))*exp(-.5*(point-handPointXYZ).transpose()*covarianceMatrix.inverse()*(point-handPointXYZ));
       // std::cout<<"hand "<<handPointXYZ<<std::endl;
    }

 //   std::cout<<"cloud "<<cloudValue<<" hand "<<handValue<<std::endl;

    overlap=handValue*cloudValue;
    return overlap;
}

/// create a single Gauss Optimization Function
handest::optimizationFunction* handest::createOptimizationFunctionGauss(void) {
	optFuncGauss.reset(new optimizationFunctionGauss());
	return optFuncGauss.get();
}
