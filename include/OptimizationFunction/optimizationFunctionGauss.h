/** @file optimizationFunctionGauss.h
 *
 * Point Cloud Optimization Function interface
 * implementation - Gauss
 */

#ifndef _OPTIMIZATION_FUNCTION_GAUSS_H_
#define _OPTIMIZATION_FUNCTION_GAUSS_H_

#include "../handest_defs.h"
#include "optimizationFunction.h"
#include <string>
#include <vector>
#include <math.h>
#include "../../dependencies/Eigen/Eigen"



using namespace handest;
using namespace Eigen;

namespace handest {
	/// create a single Point Fitting Optimization Function
	optimizationFunction* createOptimizationFunctionPF(void);
};

using namespace handest;

/// Optimization Function Point Fitting floating point
typedef handest::float_type floatGauss;

///Optimization Function implementation
	class optimizationFunctionGauss : public optimizationFunction{
	public:
		/// Fitness between model and grabbed cloud
        virtual floatGauss FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud);
	
	private:
		/// get points from hand
		void getPointsFromHand(Hand::Pose& hand);

		/// point cloud from hand
		Point3D::Cloud handPoints;

		/// covariance matrix for Gauss function
		Matrix<floatGauss,3,3>covarianceMatrix;

		/// covariance values
		floatGauss covariance_x;
		floatGauss covariance_y;
		floatGauss covariance_z;

		/// maximum sampled cloud values
		int nX;
		int nY;
		int nZ;

		/// 3D gauss function for cloud points
		floatGauss gaussFunction(Matrix<floatGauss,3,1> points, Point3D::Cloud& cloud);

		/// sampling cloud
		Matrix<floatGauss,3,1> xyzToVector(int x, int y, int z);
		
	};

#endif // _OPTIMIZATION_FUNCTION_GAUSS_H_
