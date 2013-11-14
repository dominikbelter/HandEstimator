/** @file optimizationFunction.h
 *
 * Point Cloud Optimization Function interface
 *
 */

#ifndef _OPTIMIZATION_FUNCTION_H_
#define _OPTIMIZATION_FUNCTION_H_

#include "../handest_defs.h"
#include <string>
#include <vector>

namespace handest {
	/// OptimizationFunction interface
	class optimizationFunction {
	public:

        /// Fitness between model and grabbed cloud
        virtual float_t FintnessValue(Hand::Pose& hand,Point3D::Cloud& cloud);

		/// Virtual descrutor
		virtual ~optimizationFunction() {}
	};
};

#endif // _OPTIMIZATION_FUNCTION_H_
