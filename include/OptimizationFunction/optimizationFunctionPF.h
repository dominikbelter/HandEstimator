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

using namespace handest;

/// Optimization Function Point Fitting floating point
typedef handest::float_t floatPF;

///Optimization Function implementation
	class optimizationFunctionPF : public optimizationFunction{
	public:

        /// Fitness between model and grabbed cloud
        virtual floatPF FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud) = 0;

	private:
		/// initial assignment
		/// each point from cloud is assigned to the nearest point in hand
		void initialAssignment(Hand::Pose& hand,Point3D::Cloud& cloud);

		/// find the next nearest point in hand for the given point in cloud
		int findNextNearestPoint(Hand::Pose& hand,Point3D point);

		/// check for presence of "conflict" points in hand
		bool checkForConflictHandPoints();

		/// assign point in cloud to point in hand
		void assignPointToHandPoint(Point3D cloudPoint,Point3D handPoint);

		/// assignment change planning
		void planAssignmentChange();

		/// calculate fitness value
		floatPF calculateFitnessValue();

		/// distance between 2 points
		floatPF distanceBetweenPoints(Point3D point1,Point3D point2);

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
	};

#endif // _OPTIMIZATION_FUNCTION_GAUSS_H_
