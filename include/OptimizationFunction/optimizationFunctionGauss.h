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

using namespace handest;

///Optimization Function implementation
	class optimizationFunctionGauss : public optimizationFunction{
	public:

        /// Fitness between model and grabbed cloud
        virtual float_t FitnessValue(Hand::Pose& hand,Point3D::Cloud& cloud);


	};

#endif // _OPTIMIZATION_FUNCTION_GAUSS_H_
