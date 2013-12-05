/** @file visualizerGL.h
 *
 * implementation - OpenGL Visualizer
 *
 */

#ifndef VISUALIZERGL_H_INCLUDED
#define VISUALIZERGL_H_INCLUDED

#include "visualizer.h"
#include <iostream>
#include <memory>

namespace handest {
	/// create a single visualizer (OpenGL)
	Visualizer* createVisualizerGL(void);
};

using namespace handest;

/// Visualizer implementation
class VisualizerGL : public Visualizer {
public:
	/// Pointer
	typedef std::unique_ptr<VisualizerGL> Ptr;

	/// Construction
	VisualizerGL(void);

	///Add Points
	virtual void addCloud(Point3D::Cloud& cloud, RGBA& colour) ;
		
	///Show Points
	virtual void show();

	///Clear Points
	virtual void clear();
};

#endif // VISUALIZERGL_H_INCLUDED
