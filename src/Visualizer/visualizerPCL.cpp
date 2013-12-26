#include "../include/Visualizer/visualizerPCL.h"
#include <memory>
#include <stdexcept>

using namespace handest;

/// A single instance of VisualizerPCL
VisualizerPCL::Ptr visualizerPCL;

VisualizerPCL::VisualizerPCL(void) {
    visu.reset(new pcl::visualization::PCLVisualizer("PCL visualizer"));
}

void VisualizerPCL::addCloud(Point3D::Cloud& cloud, RGBA& colour) {
    pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud;
    convertToPCL(cloud, pcl_cloud, colour);
    cout << "size: " << pcl_cloud.size() << endl;
    scenePCL.push_back(pcl_cloud);
    cout << "scene PCL size: " << scenePCL.size() << " " << scenePCL[0].size() << endl;
}

///convert to PCL format
void VisualizerPCL::convertToPCL(Point3D::Cloud& input, pcl::PointCloud<pcl::PointXYZRGBA>& output, RGBA& colour){
    output.clear();
    pcl::PointXYZRGBA point;
    for (size_t i=0;i<input.size();i++){
        point.x = input[i].position.x; point.y = input[i].position.y; point.z = input[i].position.z;
        point.r = colour.r; point.g = colour.g; point.b = colour.b; point.a = colour.a;
        output.push_back(point);
    }
}

void VisualizerPCL::show(void) const {
    for (size_t i=0;i<scenePCL.size();i++){
        std::stringstream number;
        number << i;
        std::string str = "cloud" + number.str();
        if (scenePCL[i].size()>0)
            visu->addPointCloud (scenePCL[i].makeShared(), ColorHandlerT (scenePCL[i].makeShared(), scenePCL[i][0].r, scenePCL[i][0].g, scenePCL[i][0].b), str);
    }
    visu->addCoordinateSystem();
    visu->spin ();
    while (!visu->wasStopped()){
        boost::this_thread::sleep (boost::posix_time::seconds (1));
    }
    for (size_t i=0;i<scenePCL.size();i++){
        std::stringstream number;
        number << i;
        std::string str = "cloud" + number.str();
        if (scenePCL[i].size()>0)
            visu->removePointCloud(str);
    }
}

void VisualizerPCL::clear(void) {
    scenePCL.clear();
}

handest::Visualizer* handest::createVisualizerPCL(void) {
    visualizerPCL.reset(new VisualizerPCL());
    return visualizerPCL.get();
}
