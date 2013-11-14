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
	class Optimization {
	public:

		/// Optimization with Particle Swarm Optimization Algorithm
		virtual void Optimize(Hand::Pose& hand, Point3D::Cloud& cloud) = 0;

		/// Save data to file
		virtual void SaveToFile(Hand::Pose& hand) = 0;

		/// Virtual descrutor
		virtual ~Optimization() {}
	};
};

#endif // _OPTIMIZATION_H_
