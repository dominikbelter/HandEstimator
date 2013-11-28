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

    protected:
	/// Filter name
	const std::string name;
};

#endif // FILTERPCL_H_INCLUDED
