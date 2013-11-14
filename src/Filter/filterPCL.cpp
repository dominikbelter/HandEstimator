#include "../include/Filter/filterPCL.h"
#include <memory>
#include <stdexcept>

using namespace handest;

FilterPCL::Ptr filter;

FilterPCL::FilterPCL(void) : name("Filter") {

}

const std::string& FilterPCL::getName() const {
	return name;
}

void FilterPCL::FilterScene(Point3D::Cloud& input,Point3D::Cloud& output) const {

}

