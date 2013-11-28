#include "../include/OptimizationFunction/optimizationFunctionPF.h"


float_t optimizationFunctionPF::FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud)
{
	//cloud[0].position.x;
	//hand.palm.surface[0].position.x;
	handPointsCount=hand.palm.surface.size();
	cloudPointsCount=cloud.size();

	assignmentList.resize(handPointsCount);
	assignmentHistory.resize(cloudPointsCount);
	distances.resize(cloudPointsCount);

	initialAssignment(hand,cloud);

	//findNextNearestPoint(hand,cloud[0]);

	float_t fitnessValue=calculateFitnessValue();
	return fitnessValue;
}

void optimizationFunctionPF::initialAssignment(Hand::Pose& hand,Point3D::Cloud& cloud)
{
	for(int cloudPoint=0;cloudPoint<cloudPointsCount;cloudPoint++)	
	{
		int nearestHandPoint=findNextNearestPoint(hand,cloud[cloudPoint]);
		assignPointToHandPoint(cloud[cloudPoint],hand.palm.surface[nearestHandPoint]);
	}
}

bool optimizationFunctionPF::checkForConflictHandPoints()
{

	return true;
}
		
void optimizationFunctionPF::assignPointToHandPoint(Point3D cloudPoint,Point3D handPoint)
{

}

void optimizationFunctionPF::planAssignmentChange()
{

}

int optimizationFunctionPF::findNextNearestPoint(Hand::Pose& hand,Point3D point)
{
	int nearestPoint;
	float_t distance;
	float_t nearestDistance;

	for(int i=0;i<handPointsCount;i++)
	{
		distance=distanceBetweenPoints(hand.palm.surface[i],point);
		if((i==0)||(distance<nearestDistance))
		{
			nearestDistance=distance;
			nearestPoint=i;
		}
	}
	return 0;
}

float_t optimizationFunctionPF::calculateFitnessValue()
{
	float_t value=0;
	for(int currentPoint=0;currentPoint<cloudPointsCount;currentPoint++)
	{
		value+=distances[currentPoint];
	}
	return value;
}

float_t optimizationFunctionPF::distanceBetweenPoints(Point3D point1,Point3D point2)
{
	float_t Xsq=(point2.position.x-point1.position.x)*(point2.position.x-point1.position.x);
	float_t Ysq=(point2.position.y-point1.position.y)*(point2.position.y-point1.position.y);
	float_t Zsq=(point2.position.z-point1.position.z)*(point2.position.z-point1.position.z);

	float_t distance=sqrt(Xsq+Ysq+Zsq);

	return distance;
}