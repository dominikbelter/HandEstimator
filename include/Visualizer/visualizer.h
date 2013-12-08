/** @file visualizer.h
 *
 * Point Cloud Visualizer interface
 *
 */

#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#include "../handest_defs.h"
#include <string>
#include <vector>

namespace handest {
	/// Visualizer interface
	class Visualizer {
	public:

		///Add Points
		virtual void addCloud(Point3D::Cloud& cloud, RGBA& colour) = 0;

		///Show Points
		virtual void show() const = 0;

		///Clear Points
		virtual void clear() = 0;

		/// Virtual descrutor
		virtual ~Visualizer() {}
	};
};

#endif // _VISUALIZER_H_
