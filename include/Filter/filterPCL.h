/** @file filterPCL.h
 *
 * implementation - Filter PCL
 *
 */

#ifndef FILTERPCL_H_INCLUDED
#define FILTERPCL_H_INCLUDED

#include "filter.h"
#include <iostream>
#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>

namespace handest {
	/// create a single filter
	Filter* createFilterPCL(void);
};

using namespace handest;

/// Filter implementation
class FilterPCL : public Filter {
    public:
	/// Pointer
	typedef std::unique_ptr<FilterPCL> Ptr;

	/// Construction
	FilterPCL(void);

	/// Name of the filter
	virtual const std::string& getName() const;

	/// Filter
	virtual void FilterScene(Point3D::Cloud& input, Point3D::Cloud& output) const;

    private:
	/// Filter name
	const std::string name;
};

#endif // FILTERPCL_H_INCLUDED
