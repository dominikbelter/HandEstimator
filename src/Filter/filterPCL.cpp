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
/*
void FilterPCL::FilterScene(Point3D::Cloud& input,Point3D::Cloud& output) const {
	
}*/

void FilterPCL::FilterScene(Point3D::Cloud& input,Point3D::Cloud& output) const {
	output=input;
	for(int i=0;i<sizeof(input);i++){
		if(input[i].colour.r<150) output[i].colour.r=0;
	}
}

handest::Filter* handest::createFilterPCL(void) {
	filter.reset(new FilterPCL());
	return filter.get();
}

