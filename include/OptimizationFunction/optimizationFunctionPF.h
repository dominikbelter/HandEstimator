/** @file optimizationFunctionPF.h
 *
 * Point Cloud Optimization Function interface
 * implementation - Point Fitting
 */

#ifndef _OPTIMIZATION_FUNCTION_PF_H_
#define _OPTIMIZATION_FUNCTION_PF_H_

#include "../handest_defs.h"
#include "optimizationFunction.h"
#include <string>
#include <vector>

using namespace handest;

///Optimization Function implementation
	class optimizationFunctionPF : public optimizationFunction{
	public:

        /// Fitness between model and grabbed cloud
        virtual float_t FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud) = 0;

	private:
		/// initial assignment
		/// each point from cloud is assigned to the nearest point in hand
		void initialAssignment(Hand::Pose& hand,Point3D::Cloud& cloud);

		/// find the next nearest point in hand for the given point
		int findNextNearestPoint(Hand::Pose& hand,Point3D point);

		///
		bool checkForConflictHandPoints();
		void assignPointToHandPoint(Point3D cloudPoint,Point3D handPoint);
		void planAssignmentChange();

		float_t calculateFitnessValue();

		float_t distanceBetweenPoints(Point3D point1,Point3D point2);

		int handPointsCount;
		int cloudPointsCount;

		/// list of cloud points assigned to a particular hand point
		/// no element for no assignment
		std::vector<std::vector<int>> assignmentList;

		/// list of assigned hand points (previously and currently) for a particular cloud point
		std::vector<std::vector<int>> assignmentHistory;

		/// list of distances between cloud points and current assigned hand points
		std::vector<float_t> distances;
	};

#endif // _OPTIMIZATION_FUNCTION_GAUSS_H_
