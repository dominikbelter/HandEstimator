/** @file visualizerPCL.h
 *
 * implementation - PCL Visualizer
 *
 */

#ifndef VISUALIZERPCL_H_INCLUDED
#define VISUALIZERPCL_H_INCLUDED

#include "visualizer.h"
#include <iostream>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace handest {
    /// create a single visualizer (PCL)
    Visualizer* createVisualizerPCL(void);
};

using namespace handest;

/// Visualizer implementation
class VisualizerPCL : public Visualizer {
    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> ColorHandlerT;
private:
    /// PCL cloud
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> scenePCL;
    /// PCL visualizer
    pcl::visualization::PCLVisualizer::Ptr visu;

    /// PCL point size
    int pointSize;

    ///convert to PCL format
    void convertToPCL(Point3D::Cloud& input, pcl::PointCloud<pcl::PointXYZRGBA>& output, RGBA& colour);

public:
    /// Pointer
    typedef std::unique_ptr<VisualizerPCL> Ptr;

    /// Construction
    VisualizerPCL(void);

    ///Add Points
    void addCloud(Point3D::Cloud& cloud, RGBA& colour) ;

    ///Show Points
    void show() const;

    /// Change size of visualization points
    void setPointSize(int _pointSize);

    ///Clear Points
    void clear();

};

#endif // VISUALIZERPCL_H_INCLUDED
