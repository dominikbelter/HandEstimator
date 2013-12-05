#include "../include/OptimizationFunction/optimizationFunctionPF.h"
#include <iostream>//

using namespace handest;
using namespace std;//

/// A single instance of Point Fitting Optimization Function
optimizationFunctionPF::Ptr optFuncPF;

floatPF optimizationFunctionPF::FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud)
{
	//cloud[0].position.x;
	//hand.palm.surface[0].position.x;
	handPointCount=hand.palm.surface.size();
	cloudPointCount=cloud.size();

	assignmentList.resize(handPointCount);
	assignmentHistory.resize(cloudPointCount);
	distances.resize(cloudPointCount);

	initialAssignment(hand,cloud);

	//if(checkForConflictHandPoints())
	while(checkForConflictHandPoints())
	{
		cout<<"New assignment..."<<endl;//
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

	cout<<"Cloud Point: "<<cloudPointNumber<<", Hand Point: "<<handPointNumber<<"\n";//
}

void optimizationFunctionPF::assignmentChange(Hand::Pose& hand,Point3D::Cloud& cloud)
{
	/// new assignment planning
	std::vector<std::vector<floatPF>> distanceDiff;
	std::vector<std::vector<int>> newPoints;
	std::vector<std::vector<int>> cloudPoints;
	std::vector<int> conflictPointList;
	//std::vector<int> changeVariant;

	for(int handPoint=0;handPoint<handPointCount;handPoint++)
	{
		if(assignmentList[handPoint].size()>1)
		{
			std::cout<<"Conflict Point: "<<handPoint<<"\n";
			std::vector<floatPF> currentPointDistanceDiff;
			std::vector<int> currentPointNewPoints;
			std::vector<int> currentCloudPoints;

			/// variables used to determine variant of assignment change for this point
			/// newPointContested - at least 1 point from cloud is already assigned to at least one new assignment point in hand
			/// newPointNotContested - at least 1 point from hand has no point from cloud assigned to it
			bool newPointContested=false;
			bool newPointNotContested=false;

			for(int cloudPoint=0;cloudPoint<assignmentList[handPoint].size();cloudPoint++)
			{
				int cloudPointNumber=assignmentList[handPoint][cloudPoint];
				int newPoint=findNextNearestPoint(hand,cloud,cloudPointNumber);

				if(assignmentList[newPoint].size()>0)
					newPointContested=true;
				else
					newPointNotContested=true;

				floatPF newDist=distanceBetweenPoints(cloud[cloudPointNumber],hand.palm.surface[newPoint]);
				newDist=newDist-distances[cloudPointNumber];

				if(newDist<0)
					std::cout<<"error"<<std::endl;

				currentPointDistanceDiff.push_back(newDist);
				currentPointNewPoints.push_back(newPoint);
				currentCloudPoints.push_back(cloudPointNumber);
			}

			///variant of assignment change for this point
			/// 0 - new points in hand have no point from cloud assigned to them or all new points in hand have points from cloud assigned to them
			/// 1 - mixed variant
			bool variant;

			if(newPointContested!=newPointNotContested)
				variant=0;
			else
			{
				variant=1;
				if((!newPointContested)&&(!newPointNotContested))
					std::cout<<"error"<<"\n";
			}

			/// finding cloud from point that does not change its assignment
			int minimalDiff=0;
			int i;

			switch(variant)
			{

				case 0:
					for(i=1;i<currentPointDistanceDiff.size();i++)
					{
						if(currentPointDistanceDiff[i]<currentPointDistanceDiff[minimalDiff])
							minimalDiff=i;
					}
					currentPointNewPoints[minimalDiff]=handPoint;
					break;
				case 1:
					for(i=1;i<currentPointDistanceDiff.size();i++)
					{
						if((assignmentList[currentPointNewPoints[i]].size()>0)&&(currentPointDistanceDiff[i]<currentPointDistanceDiff[minimalDiff]))
							minimalDiff=i;
					}
					currentPointNewPoints[minimalDiff]=handPoint;
					break;

			}

			//distanceDiff.push_back(currentPointDistanceDiff);
			newPoints.push_back(currentPointNewPoints);
			cloudPoints.push_back(currentCloudPoints);
			conflictPointList.push_back(handPoint);
			//changeVariant.push_back(variant);
		}
	}

	/// assignment change
	for(int conflictPoint=0;conflictPoint<conflictPointList.size();conflictPoint++)
	{

		assignmentList[conflictPointList[conflictPoint]].clear();

		for(int cloudPoint=0;cloudPoint<newPoints[conflictPoint].size();cloudPoint++)
		{
			assignPointToHandPoint(hand,cloud,cloudPoints[conflictPoint][cloudPoint],newPoints[conflictPoint][cloudPoint]);
		}
	}

}

int optimizationFunctionPF::findNextNearestPoint(Hand::Pose& hand,Point3D::Cloud& cloud,int cloudPoint)
{
	int nearestPoint;
	floatPF distance;
	floatPF nearestDistance;
	bool firstNotVisited=true;

	for(int i=0;i<handPointCount;i++)
	{
		auto notVisited=std::find(assignmentHistory[cloudPoint].begin(),assignmentHistory[cloudPoint].end(),i);

		if(notVisited==assignmentHistory[cloudPoint].end())
		{
			distance=distanceBetweenPoints(hand.palm.surface[i],cloud[cloudPoint]);

			if(firstNotVisited)
			{
				nearestDistance=distance;
				nearestPoint=i;
				firstNotVisited=false;
			}

			if(distance<nearestDistance)
			{
				nearestDistance=distance;
				nearestPoint=i;
			}
		}
	}
	return nearestPoint;
}

floatPF optimizationFunctionPF::calculateFitnessValue()
{
	floatPF value=0;
	for(int currentPoint=0;currentPoint<cloudPointCount;currentPoint++)
	{
		value+=distances[currentPoint];
	}

	value=value/cloudPointCount;

	return value;
}

floatPF optimizationFunctionPF::distanceBetweenPoints(Point3D point1,Point3D point2)
{
	floatPF Xsq=(point2.position.x-point1.position.x)*(point2.position.x-point1.position.x);
	floatPF Ysq=(point2.position.y-point1.position.y)*(point2.position.y-point1.position.y);
	floatPF Zsq=(point2.position.z-point1.position.z)*(point2.position.z-point1.position.z);

	floatPF distance=sqrt(Xsq+Ysq+Zsq);

	return distance;
}

handest::optimizationFunction* handest::createOptimizationFunctionPF(void) {
	optFuncPF.reset(new optimizationFunctionPF());
	return optFuncPF.get();
}
