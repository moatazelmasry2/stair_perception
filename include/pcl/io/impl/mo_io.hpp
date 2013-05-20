/*
 * mo_io.hpp
 *
 *  Created on: Feb 6, 2013
 *      Author: elmasry
 */

#ifndef MO_IO_HPP_
#define MO_IO_HPP_

#include <pcl/point_cloud.h>
#include "pcl/common/color.h"

#include "pcl/models/localmodel.h"

template<typename PointT>
void pcl::io::saveModel (const std::string file_name, const LocalModel<PointT>& model)
{
  typedef Eigen::aligned_allocator<PointT> Alloc;
  typedef std::vector<Step<PointT>, Alloc> StepsVector;
  typedef std::vector<pcl::Plane3D<PointT>, Alloc> Plane3DVector;

  typename pcl::PointCloud<PointT>::Ptr outCloud (new pcl::PointCloud<PointT>);

  for (size_t i = 0; i < model.steps.size (); i++)
  {
    float color = pcl::color::getRandomColor ();
    const Step<PointT>& step = model.steps[i];
    if (step.hasRiser ())
    {
      typename pcl::PointCloud<PointT>::Ptr cloud = step.getRiser ().getCloud ();
      colorCloud<PointT> (*cloud, color);
      outCloud->insert (outCloud->end (), cloud->begin (), cloud->end ());
    }

    if (step.hasTread ())
    {
      typename pcl::PointCloud<PointT>::Ptr cloud = step.getTread ().getCloud ();
      colorCloud<PointT> (*cloud, color);
      outCloud->insert (outCloud->end (), cloud->begin (), cloud->end ());
    }

  }
  if (outCloud->size () > 0)
  {
    pcl::io::savePCDFileBinary (file_name, *outCloud);
  }
  else
  {
    std::cerr << "cannot save model. cloud is empty\n" << std::endl;
  }
}

template<typename PointT>
void pcl::io::saveMoPcd (const char* name, pcl::PointCloud<PointT> cloud)
{
  if (cloud.size() > 0) {
    pcl::io::savePCDFileASCII(name, cloud);
  } else {
    std::cerr << "cloud: " << name << "is empty" << std::endl;
  }
}

#endif /* MO_IO_HPP_ */
