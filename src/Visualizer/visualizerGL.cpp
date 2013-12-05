#include "../include/Visualizer/visualizerGL.h"
#include <memory>
#include <stdexcept>

using namespace handest;

/// A single instance of VisualizerGL
VisualizerGL::Ptr visualizer;

VisualizerGL::VisualizerGL(void) {

}

void VisualizerGL::addCloud(Point3D::Cloud& cloud, RGBA& colour) {

}

void VisualizerGL::show(void) {

}
void VisualizerGL::clear(void) {

}

handest::Visualizer* handest::createVisualizerGL(void) {
	visualizer.reset(new VisualizerGL());
	return visualizer.get();
}
