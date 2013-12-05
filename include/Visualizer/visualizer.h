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
		virtual void addCloud(Point3D::Cloud& cloud, RGBA& colour);
		
		///Show Points
		virtual void show();

		///Clear Points
		virtual void clear();

		/// Virtual descrutor
		virtual ~Visualizer() {}
	};
};

#endif // _VISUALIZER_H_
