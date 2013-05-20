/*
 * demo.cpp
 *
 *  Created on: Jul 5, 2012
 *      Author: elmasry
 *
 *      Takes in a folder of pcds and infinitly reads them and send them to the stair detection class
 */

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
//#include <pcl_visualization/pcl_visualizer.h>

#include "pcl/common/fhg_point_types.h"
#include "pcl/utils/pointcloud_utils.h"
#include "pcl/types/plane3d.h"
#include "pcl/common/color.h"

#include "pcl/common/fhg_point_types.h"

#include <pcl/features/normal_3d.h>

#include "pcl/opencv/cvutils.h"
#include "pcl/opencv/edgesdetector.h"

#include "pcl/models/localmodel.h"
#include "pcl/apps/stairdetectiondemo.h"

#include "pcl/io/mo_io.h"
#include "pcl/surface/surface_utils.h"

//typedef pcl::PointXYZRGB PointIn;
//typedef pcl::PointXYZRGBNormal PointOut;

typedef pcl::PointMoXYZRGB PointIn;
typedef pcl::PointMoXYZRGBNormal PointOut;
pcl::PointCloud<PointIn> cloud;

pcl::StairDetectionDemo<PointIn, PointOut> demo;
int numIterations = 0;
void renderCloud (std::string path)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<PointIn>::Ptr inCloudMo (new pcl::PointCloud<PointIn>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path.c_str (), *inCloud) == -1)  //* load the file
  {
    char c[200];
    sprintf (c, "Couldn't read file %s\n", path.c_str ());
    throw std::runtime_error (c);
  }
  printf ("processing pcd file: %s\n", path.c_str ());
  pcl::copyPointCloud (*inCloud, *inCloudMo);
//  printf("pcds id=\n");
  for (size_t i = 0; i < inCloud->size (); i++)
  {
    (*inCloudMo)[i].id = (int) i;
  }

  size_t index = path.find_last_of ("/");
  std::string s = path.substr (index + 1, path.length () - index);
  std::string rawPath = s.substr (0, s.length () - 4);

//  demo.setInputCloud (inCloud);
  demo.setInputCloud (inCloudMo);
  demo.setCloudName (rawPath);
//  printf("rawPath=%s\n", rawPath.c_str());
  pcl::PointCloud<PointIn>::Ptr out = demo.compute ();

  std::string outPath = s;

  if (out->size () > 0)
  {
    outPath.replace (s.length () - 4, 4, "_out.vtk");
//    pcl::PointCloud<PointOut>::ConstPtr ptr2 = out;
//    pcl::io::savePolygonFileVTK (outPath.c_str (), *pcl::surface::getMesh<PointIn, PointOut>(out) );
    outPath.replace (outPath.length () - 4, 4, ".pcd");
    char f[50];
    sprintf (f, "out_%d.pcd", numIterations);
//    pcl::io::savePCDFileBinary (outPath.c_str (), *out);
    pcl::io::saveMoPcd (f, *out);
  }
  //pcl::io::savePCDFileBinary ("steps.pcd", *out);

  {
    pcl::PointCloud<PointOut>::Ptr planesCloud = demo.getPlanesCloud ();
    std::string planesPath = s;
    planesPath.replace (s.length () - 4, 4, "_planes.pcd");
    char f[50];
    sprintf(f, "planes_%d.pcd", numIterations);
    if (planesCloud->size () > 0)
    {
      pcl::io::saveMoPcd (f, *planesCloud);
    }
  }
  //pcl::io::savePCDFileBinary ("planes.pcd", *planesCloud);

  pcl::PointCloud<PointOut>::Ptr planesCloud2 = demo.getRawPlanesCloud ();
  if (planesCloud2->size () > 0)
  {
    char f[50];
    sprintf(f, "rawplanes_%d.pcd", numIterations);
    pcl::io::saveMoPcd (f, *planesCloud2);
  }
//  pcl::io::savePCDFileBinary ("rawplanes.pcd", *planesCloud2);

  char f[50];
  sprintf (f, "globalmodel_%d.pcd", numIterations++);
//  pcl::io::saveMoPcd(f, *demo.getGlobalModel ().getModelCloud());
  pcl::io::saveModel<PointOut> (std::string (f), (pcl::LocalModel<PointOut>) demo.getGlobalModel ());
}
int main (int argc, char **argv)
{
  //std::cout << "numinputs=" << argc - 1 << std::endl;
  for (size_t i = 1; i < (size_t) argc; i++)
  {
    std::string p (argv[i]);
    renderCloud (argv[i]);
  }
  return 0;
}

