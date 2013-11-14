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


	};

#endif // _OPTIMIZATION_FUNCTION_GAUSS_H_
