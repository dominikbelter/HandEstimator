/** @file optimizationFunctionPF.h
 *
 * Point Cloud Optimization Function interface
 * implementation - Point Fitting
 */

#ifndef _OPTIMIZATION_FUNCTION_PF_H_
#define _OPTIMIZATION_FUNCTION_PF_H_

#include "../handest_defs.h"
#include "optimizationFunction.h"
#include <math.h>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <memory>

namespace handest {
	/// create a single Point Fitting Optimization Function
	optimizationFunction* createOptimizationFunctionPF(void);
};

using namespace handest;

/// Optimization Function Point Fitting floating point
typedef handest::float_t floatPF;

///Optimization Function implementation
	class optimizationFunctionPF : public optimizationFunction{
	public:
		/// Pointer
		typedef std::unique_ptr<optimizationFunctionPF> Ptr;

        /// Fitness between model and grabbed cloud
        virtual floatPF FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud);

	private:
		/// initial assignment
		/// each point from cloud is assigned to the nearest point in hand
		void initialAssignment(Point3D::Cloud& cloud);

		/// find the next nearest point in hand for the given point in cloud
		int findNextNearestPoint(Point3D::Cloud& cloud,int cloudPoint);

		/// check for presence of "conflict" points in hand
		bool checkForConflictHandPoints();

		/// assign point in cloud to point in hand
		void assignPointToHandPoint(Point3D::Cloud& cloud,int cloudPointNumber,int handPointNumber);

		/// plan and make new assignment
		void assignmentChange(Point3D::Cloud& cloud);

		/// calculate fitness value
		floatPF calculateFitnessValue();

		/// distance between 2 points
		floatPF distanceBetweenPoints(Point3D point1,Point3D point2);

		/// get points from hand
		void getPointsFromHand(Hand::Pose& hand);

		/// number of points in hand
		int handPointCount;

		/// number of points in cloud
		int cloudPointCount;

		/// vector of points in cloud assigned to a particular point in hand
		/// no element for no assignment
		std::vector<std::vector<int>> assignmentList;

		/// vector of assigned points in hand (previously and currently) for a particular point in cloud
		std::vector<std::vector<int>> assignmentHistory;

		/// vector of distances between points in cloud and points in hand (currently assigned)
		std::vector<floatPF> distances;

		/// point cloud from hand
		Point3D::Cloud handPoints;
	};

#endif // _OPTIMIZATION_FUNCTION_GAUSS_H_
