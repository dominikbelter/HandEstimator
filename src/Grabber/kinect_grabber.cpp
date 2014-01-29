#include "../include/Grabber/kinect_grabber.h"
#include <memory>
#include <stdexcept>

#include <iostream>
#include <string>
#include <sstream>



using namespace handest;

/// A single instance of Kinect grabber
KinectGrabber::Ptr grabber;

KinectGrabber::KinectGrabber(void) : name("Kinect Grabber") {
    frame_no = 0;
}


const std::string& KinectGrabber::getName() const {
	return name;
}


void KinectGrabber::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
            cloud_temp=*cloud;
            frame_no=1;
}

void KinectGrabber::run (void)
{
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

   	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
   	boost::bind (&KinectGrabber::cloud_cb_, this, _1);
   	interface->registerCallback (f);
   	interface->start ();
   	while (frame_no<1) //oczekiwanie na przechwycenie pojedynczej chmury
   	{
		 
	}
  	  interface->stop ();
}



void KinectGrabber::getCloud(Point3D::Cloud& current_cloud) const {
    
	Point3D point;
	cout << "rozmiar: " << cloud_temp.points.size() << endl;
	for(size_t i=0;i<cloud_temp.points.size();i++)    
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


void KinectGrabber::LoadFromFile(std::string path)     // use instead of grabber->run to load a PCL cloud
{ 
	pcl::io::loadPCDFile(path, cloud_temp); 
}


handest::Grabber* handest::createGrabberKinect(void) 
{
	grabber.reset(new KinectGrabber());
	return grabber.get();
}
