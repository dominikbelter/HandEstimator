#include "../include/Grabber/kinect_grabber.h"
#include <memory>
#include <stdexcept>

#include <iostream>
#include <string>
#include <sstream>

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/io/openni_grabber.h>
//#include <pcl/visualization/cloud_viewer.h>

using namespace handest;

/// A single instance of Kinect grabber
KinectGrabber::Ptr grabber;

KinectGrabber::KinectGrabber(void) : name("Kinect Grabber") {
    //viewer = pcl::visualization::CloudViewer("dfssd");
}


//KinectGrabber () : viewer ("PCL Viewer") // nie wiem co to jest i czy jest potrzebne
// {}

const std::string& KinectGrabber::getName() const {
	return name;
}


void KinectGrabber::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
     // if (!viewer.wasStopped())
      //{
            //viewer.showCloud (cloud);
            //std::stringstream out;
            // out << frames_saved;
            //std::string name = OUT_DIR + "cloud" + out.str() + ".pcd";    // zapis chmury do pliku
            // pcl::io::savePCDFileASCII( name, *cloud );

            //int n=(cloud->width*cloud->height);
            //
            //std::cout << cloud->size() << endl;
            cloud_temp=*cloud;
            cout << "rozmiar1: " << cloud_temp.points.size() << endl;
    
            frame_no=1;
	    
	    cout << "jcb = " << cout << (int)frame_no << endl;
     // }
 
}

void KinectGrabber::run (void)
{
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
            boost::bind (&KinectGrabber::cloud_cb_, this, _1);

    interface->registerCallback (f);
            interface->start ();
std::cout << "dfs1\n";
            while (frame_no<1) //oczekiwanie na przechwycenie pojedynczej chmury
            {
	      cout << "j = " << cout << (int)frame_no << endl;
            }
std::cout << "dfs3\n";
            interface->stop ();
}



void KinectGrabber::getCloud(Point3D::Cloud& current_cloud) const {
    
	Point3D point;
	cout << "rozmiar: " << cloud_temp.points.size() << endl;
	for(size_t i=0;i<cloud_temp.points.size();i++)    // tutaj moze byc zla ilosc punktow
            {
                point.colour.r = cloud_temp.points[i].r; // zamiana formatu chmury z PCD na nasz format
                point.colour.g = cloud_temp.points[i].g;
                point.colour.b = cloud_temp.points[i].b;
                point.colour.a = cloud_temp.points[i].a;
                point.position.v[0] = cloud_temp.points[i].x;
                point.position.v[1] = cloud_temp.points[i].y;
                point.position.v[2] = cloud_temp.points[i].z;
                current_cloud.push_back(point);
            }

}

//void KinectGrabber::grab(void) {
   // Point3D point;
   // point.colour.r = 255; point.colour.g = 0; point.colour.b = 0; point.colour.a = 255;
   // point.position.v[0] = 1.2; point.position.v[1] = 3.4; point.position.v[2] = 5.6;
    //cloud.push_back(point);
   // run();
//}


handest::Grabber* handest::createGrabberKinect(void) {
	grabber.reset(new KinectGrabber());
	return grabber.get();
}
