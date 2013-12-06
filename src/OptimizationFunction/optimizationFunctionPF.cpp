#include "../include/OptimizationFunction/optimizationFunctionPF.h"

using namespace handest;

/// A single instance of Point Fitting Optimization Function
optimizationFunctionPF::Ptr optFuncPF;

floatPF optimizationFunctionPF::FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud)
{
	getPointsFromHand(hand);

	handPointCount=handPoints.size();
	cloudPointCount=cloud.size();

	if(cloudPointCount>handPointCount)
	{
		std::cout<<"Error in OptimizationFunctionPF. Hand must contain more points than grabbed cloud."<<"\n";
		exit(1);
	}

	assignmentList.resize(handPointCount);
	assignmentHistory.resize(cloudPointCount);
	distances.resize(cloudPointCount);

	initialAssignment(cloud);

	while(checkForConflictHandPoints())
	{
		//std::cout<<"New assignment..."<<"\n";
		assignmentChange(cloud);
	}

	floatPF fitnessValue=calculateFitnessValue();
	return fitnessValue;
}

void optimizationFunctionPF::initialAssignment(Point3D::Cloud& cloud)
{
	for(int cloudPoint=0;cloudPoint<cloudPointCount;cloudPoint++)
	{
		int nearestHandPoint=findNextNearestPoint(cloud,cloudPoint);
		assignPointToHandPoint(cloud,cloudPoint,nearestHandPoint);
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

void optimizationFunctionPF::assignPointToHandPoint(Point3D::Cloud& cloud,int cloudPointNumber,int handPointNumber)
{
	assignmentList[handPointNumber].push_back(cloudPointNumber);
	assignmentHistory[cloudPointNumber].push_back(handPointNumber);

	floatPF dist=distanceBetweenPoints(cloud[cloudPointNumber],handPoints[handPointNumber]);
	distances[cloudPointNumber]=dist;

	//std::cout<<"Cloud Point: "<<cloudPointNumber<<", Hand Point: "<<handPointNumber<<"\n";
}

void optimizationFunctionPF::assignmentChange(Point3D::Cloud& cloud)
{
	/// new assignment planning
	std::vector<std::vector<floatPF>> distanceDiff;
	std::vector<std::vector<int>> newPoints;
	std::vector<std::vector<int>> cloudPoints;
	std::vector<int> conflictPointList;

	for(int handPoint=0;handPoint<handPointCount;handPoint++)
	{
		if(assignmentList[handPoint].size()>1)
		{
			//std::cout<<"Conflict Point: "<<handPoint<<"\n";
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
				int newPoint=findNextNearestPoint(cloud,cloudPointNumber);

				if(assignmentList[newPoint].size()>0)
					newPointContested=true;
				else
					newPointNotContested=true;

				floatPF newDist=distanceBetweenPoints(cloud[cloudPointNumber],handPoints[newPoint]);
				newDist=newDist-distances[cloudPointNumber];

				if(newDist<0)
				{
					std::cout<<"Error in OptimizationFunctionPF. In assignmentChange: difference between distances is less than 0."<<"\n";
					exit(2);
				}

				currentPointDistanceDiff.push_back(newDist);
				currentPointNewPoints.push_back(newPoint);
				currentCloudPoints.push_back(cloudPointNumber);
			}

			/// variant of assignment change for this point
			/// 0 - new points in hand have no point from cloud assigned to them or all new points in hand have points from cloud assigned to them
			/// 1 - mixed variant
			bool variant;

			if(newPointContested!=newPointNotContested)
				variant=0;
			else
			{
				variant=1;
				if((!newPointContested)&&(!newPointNotContested))
				{
					std::cout<<"Error in OptimizationFunctionPF. In assignmentChange: invalid change variant"<<"\n";
					exit(3);
				}
			}

			/// finding cloud from point that does not change its assignment
			int maximalDiff;

			switch(variant)
			{

				case 0:
					maximalDiff=0;
					for(int i=1;i<currentPointDistanceDiff.size();i++)
					{
						if(currentPointDistanceDiff[i]>currentPointDistanceDiff[maximalDiff])
							maximalDiff=i;
					}
					currentPointNewPoints[maximalDiff]=handPoint;
					break;
				case 1:
					bool firstPointFound=false;
					for(int i=0;i<currentPointDistanceDiff.size();i++)
					{
						if(assignmentList[currentPointNewPoints[i]].size()>0)
						{
							if(firstPointFound)
							{
								if(currentPointDistanceDiff[i]>currentPointDistanceDiff[maximalDiff])
									maximalDiff=i;
							}
							else
							{
								maximalDiff=i;
								firstPointFound=true;
							}
						}
					}
					currentPointNewPoints[maximalDiff]=handPoint;
					break;

			}

			newPoints.push_back(currentPointNewPoints);
			cloudPoints.push_back(currentCloudPoints);
			conflictPointList.push_back(handPoint);
		}
	}

	/// assignment change
	for(int conflictPoint=0;conflictPoint<conflictPointList.size();conflictPoint++)
	{

		assignmentList[conflictPointList[conflictPoint]].clear();

		for(int cloudPoint=0;cloudPoint<newPoints[conflictPoint].size();cloudPoint++)
		{
			assignPointToHandPoint(cloud,cloudPoints[conflictPoint][cloudPoint],newPoints[conflictPoint][cloudPoint]);
		}
	}

}

int optimizationFunctionPF::findNextNearestPoint(Point3D::Cloud& cloud,int cloudPoint)
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
			distance=distanceBetweenPoints(handPoints[i],cloud[cloudPoint]);

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

void optimizationFunctionPF::getPointsFromHand(Hand::Pose& hand)
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

handest::optimizationFunction* handest::createOptimizationFunctionPF(void) {
	optFuncPF.reset(new optimizationFunctionPF());
	return optFuncPF.get();
}
