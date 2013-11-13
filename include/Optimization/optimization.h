/** @file optimization.h
 *
 * Optimization interface
 *
 */

#ifndef _OPTIMIZATION_H_
#define _OPTIMIZATION_H_

#include "../handest_defs.h"


namespace handest {
	/// Optimization interface
	class OptimizationPSO {
	public:

		/// Optimization with Particle Swarm Optimization Algorithm
		virtual void Optimization(Hand::Pose& hand, Point3D::Cloud& cloud) = 0;

		/// Save data to file
		virtual void SaveToFile(Hand::Pose& hand) = 0;

		/// Virtual descrutor
		virtual ~OptimizationPSO() {}
	};
};

#endif // _OPTIMIZATION_H_
