/** @file optimizationFunctionGauss.h
 *
 * Point Cloud Optimization Function interface
 * implementation - Gauss
 */

#ifndef _OPTIMIZATION_FUNCTION_GAUSS_H_
#define _OPTIMIZATION_FUNCTION_GAUSS_H_

#include "../handest_defs.h"
#include "optimizationFunction.h"
#include <math.h>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <memory>
#include "../../dependencies/Eigen/Eigen"

namespace handest {
	/// create a single Gauss Optimization Function
	optimizationFunction* createOptimizationFunctionGauss(void);
};

using namespace handest;
using namespace Eigen;

/// Optimization Function Point Fitting floating point
typedef handest::float_type floatGauss;

///Optimization Function implementation
	class optimizationFunctionGauss : public optimizationFunction{
	public:
	    /// Pointer
		typedef std::unique_ptr<optimizationFunctionGauss> Ptr;

		/// Fitness between model and grabbed cloud
        virtual floatGauss FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud);
        ///DB zaginal auto-pointer i podobnie tworzenie obiektu w pliku cpp

	private:
		/// get points from hand
		void getPointsFromHand(Hand::Pose& hand);

		/// point cloud from hand
		Point3D::Cloud handPoints;

		/// covariance matrix for Gauss function
		Matrix<floatGauss,3,3>covarianceMatrix;

        /// covariance values ///DB lepiej tablica
		floatGauss covariance_x;
		floatGauss covariance_y;
		floatGauss covariance_z;

        /// maximum sampled cloud values ///DB lepiej tablica
		floatGauss nX;
		floatGauss nY;
		floatGauss nZ;

		/// 3D gauss function for cloud points
		floatGauss gaussFunction(Matrix<floatGauss,3,1> point, Point3D::Cloud& cloud);

		/// sampling cloud
		Matrix<floatGauss,3,1> xyzToVector(floatGauss x, floatGauss y, floatGauss z);

	};

#endif // _OPTIMIZATION_FUNCTION_GAUSS_H_
