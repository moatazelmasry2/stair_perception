/*
 * normalsvis.cpp
 *
 *  Created on: Mar 12, 2013
 *      Author: elmasry
 */
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pcl/common/pcl_commons.h"

typedef pcl::PointXYZRGB PointIn;
//typedef pcl::PointXYZRGB PointOut;

int main (int argc, char **argv)
{
  pcl::PointCloud<PointIn>::Ptr cloud (new pcl::PointCloud<PointIn>);

  if (argc < 2)
  {
    fprintf (stderr, "missing input image \n");
    return 1;
  }

  if (pcl::io::loadPCDFile<PointIn> (argv[1], *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file %s\n", argv[1]);
    return -1;
  }

  pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::itnegralImageNormal<PointIn> (*cloud);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0,0,0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointIn> rgb (cloud);
    viewer->addPointCloud<PointIn> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 50, 0.1, "normals");

//    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
