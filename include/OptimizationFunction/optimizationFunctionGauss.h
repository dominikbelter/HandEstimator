/** @file optimizationFunctionGauss.h
 *
 * Point Cloud Optimization Function interface
 * implementation - Gauss
 */

#ifndef _OPTIMIZATION_FUNCTION_GAUSS_H_
#define _OPTIMIZATION_FUNCTION_GAUSS_H_

#include "../handest_defs.h"
#include "optimizationFunction.h"
#include <string>
#include <vector>

namespace handest {
    /// create Gauss optimization function
    optimizationFunction* create
}

namespace handest {
	/// OptimizationFunction interface
	class optimizationFunction {
	public:

        /// Fitness between model and grabbed cloud
        virtual float_t FintnessValue(Hand::Pose& hand,Point3D::Cloud& cloud);

		/// Virtual descrutor
		virtual ~optimizationFunction() {}
	};
};

#endif // _OPTIMIZATION_FUNCTION_H_

/*
namespace handest {
	/// create a single grabber (Kinect)
	Grabber* createGrabberKinect(void);
};

using namespace handest;

/// Grabber implementation
class KinectGrabber : public Grabber {
    public:
	/// Pointer
	typedef std::unique_ptr<KinectGrabber> Ptr;

	/// Construction
	KinectGrabber(void);

	/// Name of the grabber
	virtual const std::string& getName() const;

	/// Returns the current point cloud
	virtual void getCloud(Point3D::Cloud& current_cloud) const;

	/// Grab point cloud
	virtual void grab();

    protected:
	/// RGBZXYZ Point cloud
	Point3D::Cloud cloud;
	/// Grabber name
	const std::string name;
};

*/
