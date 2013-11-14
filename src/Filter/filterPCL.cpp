#include "../include/Filter/filter.h"
#include <memory>
#include <stdexcept>

using namespace handest;

Filter::Ptr filter;

Filter::Filter(void) : name("Filter") {

}

const std::string& Filter::getName() const {
	return name;
}

void FilterScene(Point3D::Cloud& input,Point3D::Cloud& output) const {
}

