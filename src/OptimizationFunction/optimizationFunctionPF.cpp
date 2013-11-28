#include "../include/OptimizationFunction/optimizationFunctionPF.h"


handest::float_t optimizationFunctionPF::FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud)
{
	//cloud[0].position.x;
	//hand.palm.surface[0].position.x;
	handPointCount=hand.palm.surface.size();
	cloudPointCount=cloud.size();

	assignmentList.resize(handPointCount);
	assignmentHistory.resize(cloudPointCount);
	distances.resize(cloudPointCount);

	initialAssignment(hand,cloud);

	while(checkForConflictHandPoints())
	{
		assignmentChange(hand,cloud);
	}

	floatPF fitnessValue=calculateFitnessValue();
	return fitnessValue;
}

void optimizationFunctionPF::initialAssignment(Hand::Pose& hand,Point3D::Cloud& cloud)
{
	for(int cloudPoint=0;cloudPoint<cloudPointCount;cloudPoint++)	
	{
		int nearestHandPoint=findNextNearestPoint(hand,cloud,cloudPoint);
		assignPointToHandPoint(hand,cloud,cloudPoint,nearestHandPoint);
	}
}

bool optimizationFunctionPF::checkForConflictHandPoints()
{
	for(int handPoint=0;handPoint<handPointCount;handPoint++)
	{
		if(assignmentList[handPoint].size()>1)
		{
			return true;
		}	
	}
	return false;
}
		
void optimizationFunctionPF::assignPointToHandPoint(Hand::Pose& hand,Point3D::Cloud& cloud,int cloudPointNumber,int handPointNumber)
{
	assignmentList[handPointNumber].push_back(cloudPointNumber);
	assignmentHistory[cloudPointNumber].push_back(handPointNumber);

	floatPF dist=distanceBetweenPoints(cloud[cloudPointNumber],hand.palm.surface[handPointNumber]);
	distances[cloudPointNumber]=dist;
}

void optimizationFunctionPF::assignmentChange(Hand::Pose& hand,Point3D::Cloud& cloud)
{
	for(int handPoint=0;handPoint<handPointCount;handPoint++)
	{
		if(!assignmentList[handPoint].size()>1)
		{
			std::vector<floatPF> distanceDiff;
			std::vector<int> newPoints;
			int changeVariant;
			for(int cloudPoint=0;cloudPoint<assignmentList[handPoint].size();cloudPoint++)
			{
				int cloudPointNumber=assignmentList[handPoint][cloudPoint];
				int newPoint=findNextNearestPoint(hand,cloud,cloudPointNumber);

				floatPF newDist=distanceBetweenPoints(cloud[cloudPointNumber],hand[newPoint]);
				newDist=newDist-distances[cloudPointNumber];

				distanceDiff.push_back(newDist);
				newPoints.push_back(newPoint);
			}

		}
	}
}

int optimizationFunctionPF::findNextNearestPoint(Hand::Pose& hand,Point3D::Cloud& cloud,int cloudPoint)
{
	int nearestPoint;
	floatPF distance;
	floatPF nearestDistance;

	for(int i=0;i<handPointCount;i++)
	{
		auto notVisited=std::find(assignmentHistory[cloudPoint].begin(),assignmentHistory[cloudPoint].end(),i);

		if(notVisited==assignmentHistory[cloudPoint].end())
		{
			distance=distanceBetweenPoints(hand.palm.surface[i],cloud[cloudPoint]);
			if((i==0)||(distance<nearestDistance))
			{
				nearestDistance=distance;
				nearestPoint=i;
			}
		}
	}
	return nearestPoint;
}

handest::float_t optimizationFunctionPF::calculateFitnessValue()
{
	floatPF value=0;
	for(int currentPoint=0;currentPoint<cloudPointCount;currentPoint++)
	{
		value+=distances[currentPoint];
	}
	return value;
}

handest::float_t optimizationFunctionPF::distanceBetweenPoints(Point3D point1,Point3D point2)
{
	floatPF Xsq=(point2.position.x-point1.position.x)*(point2.position.x-point1.position.x);
	floatPF Ysq=(point2.position.y-point1.position.y)*(point2.position.y-point1.position.y);
	floatPF Zsq=(point2.position.z-point1.position.z)*(point2.position.z-point1.position.z);

	floatPF distance=sqrt(Xsq+Ysq+Zsq);

	return distance;
}