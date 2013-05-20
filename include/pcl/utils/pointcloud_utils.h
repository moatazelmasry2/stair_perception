/*
 * pointcloud_utils.h
 *
 *  Created on: Jun 25, 2012
 *      Author: elmasry
 */

#ifndef POINTCLOUD_UTILS_H_
#define POINTCLOUD_UTILS_H_

#include <string>
#include <iostream>
#include <fstream>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/common/point_common.h>

#include "pcl/registration/transforms.h"
#include "pcl/common/color.h"

using namespace std;

namespace pcl
{
  template<typename PointT>
    void
    colorCloud (pcl::PointCloud<PointT>& cloud, float color)
    {
      for (size_t i = 0; i < cloud.size (); i++)
      {
        PointT& p = cloud.points[i];
        p.rgb = color;
      }
    }

  template<typename PointT>
    void
    colorCloud (typename pcl::PointCloud<PointT>::Ptr cloud, float color)
    {
      for (size_t i = 0; i < cloud->size (); i++)
      {
        PointT& p = cloud->points[i];
        p.rgb = color;
      }
    }

  /**
   * rotates camera coordinate to world coordinate
   */
  template<typename PointT>
    void
    cameraToworld (pcl::PointCloud<PointT>& cloud)
    {
      Eigen::Quaternion<float> q;
      q = Eigen::Quaternion<float> (-0.500398163355, 0.499999841466, -0.499601836645, 0.499999841466);
      pcl::transformPointCloud (cloud, cloud, Eigen::Vector3f (0, 0, 0), q);
    }

  /**
   * rotates world coordinate to camera coordinates
   */
  template<typename PointT>
    void
    worldToCamera (pcl::PointCloud<PointT>& cloud)
    {
      Eigen::Quaternion<float> q;
      q = Eigen::Quaternion<float> (0.500398163355, -0.499999841466, 0.499601836645, -0.499999841466);
      pcl::transformPointCloud (cloud, cloud, Eigen::Vector3f (0, 0, 0), q);
      //-0.500398163355, 0.499999841466, -0.499601836645, 0.499999841466);
      //: -0.499999841466, y: 0.499601836645, z: -0.499999841466, w: 0.500398163355

    }

  /**
   *
   * \param angle in degrees
   */
  template<typename PointT>
    void
    rotatePointCloud (pcl::PointCloud<PointT>& cloud, Eigen::Quaternion<float> q)
    {
      pcl::transformPointCloud (cloud, cloud, Eigen::Vector3f (0, 0, 0), q);
    }

  /**
   *
   * \param angle in degrees
   */
  template<typename PointT>
    void
    rotatePointCloud (pcl::PointCloud<PointT>& cloud, double angle, Eigen::Vector3f axis)
    {
      Eigen::Quaternion<float> q;
      q = Eigen::AngleAxis<float> (angle * M_PI / 180, axis);
      rotatePointCloud (cloud, q);
    }

  /**
   *  multiply each point in pointcloud by 1000
   */
  template<typename PointT>
    void
    mult1000 (pcl::PointCloud<PointT>& source)
    {
      typedef Eigen::aligned_allocator<PointT> Alloc;
      typedef std::vector<PointT, Alloc> PointsVector;
      for (typename PointsVector::iterator it = source.points.begin (); it != source.points.end (); it++)
      {
        PointT & p = *it;
        if (isnan (p.x) || isnan (p.y) || isnan (p.z))
        {
          p.x = 0.0;
          p.y = 0.0;
          p.z = 0.0;
        }
        else
        {
          p.x *= 1000;
          p.y *= 1000;
          p.z *= 1000;
        }
      }
    }

  /**
   *  multiply each point in pointcloud by 1000
   */
  template<typename PointT>
    void
    div1000 (pcl::PointCloud<PointT>& source)
    {
      typedef Eigen::aligned_allocator<PointT> Alloc;
      typedef std::vector<PointT, Alloc> PointsVector;
      for (typename PointsVector::iterator it = source.points.begin (); it != source.points.end (); it++)
      {
        PointT & p = *it;
        if (isnan (p.x) || isnan (p.y) || isnan (p.z))
        {
          p.x = 0.0;
          p.y = 0.0;
          p.z = 0.0;
        }
        else
        {
          p.x /= 1000;
          p.y /= 1000;
          p.z /= 1000;
        }
      }
    }
  template<typename PointT>
    void
    concatePointClouds (const pcl::PointCloud<PointT>& cloud, pcl::PointCloud<PointT>& outCloud)
    {
      outCloud.insert (outCloud.end (), cloud.begin (), cloud.end ());
      outCloud.width = outCloud.points.size ();
      outCloud.height = 1;
    }

  template<typename PointIn, typename PointOut>
    void
    copyPointCloud (const typename pcl::PointCloud<PointIn>::ConstPtr cloudIn, pcl::PointCloud<PointOut>& cloudOut)
    {
      cloudOut.clear ();
      cloudOut.insert (cloudOut.end (), cloudIn.begin (), cloudIn.end ());
      cloudOut.width = cloudIn.width;
      cloudOut.height = cloudIn.height;
    }

  /**
   * returns the unsigned distance between two points on one compenet, i.e. x,y, or z
   */
  template<typename PointT>
    float
    distCompPoint (const PointT& p1, const PointT& p2, int component)
    {
    //std::cout << "p1=" << p1 << ", p2=" << p2 << std::endl;
      switch (component)
      {
        case 0:
          return fabs (p1.x - p2.x);
        case 1:
          return fabs (p1.y - p2.y);
        case 2:
          return fabs (p1.z - p2.z);
        default:
          throw "component must be between 0-2";
      }
    }

  /**
   * Converts PointMo to PointXYZRGB
   */
  template<typename PointIn, typename PointOut>
    typename pcl::PointCloud<PointOut>::Ptr
    pointmo2RGB (typename pcl::PointCloud<PointIn>::Ptr input)
    {
      typename pcl::PointCloud<PointOut>::Ptr out (new pcl::PointCloud<PointOut>);
      for (size_t i = 0; i < input->points.size (); i++)
      {
        PointOut pOut;
        PointIn& pIn = input->points[i];
        pOut.x = pIn.x;
        pOut.y = pIn.y;
        pOut.z = pIn.z;
        pOut.rgb = pIn.rgb;
        out->push_back (pOut);
      }
      return out;
    }

  /**
   * converts an indices matrix, where each index entry is either 1 or -1. 1 means a valid point, take it, invalid otherwise
   */
  template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr
    indices2Cloud (const pcl::PointCloud<PointT>& inCloud, const std::vector<std::vector<int> >& indices)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
      for (size_t i = 0; i < indices.size (); i++)
      {
        std::vector<int> col = indices[i];
        for (size_t j = 0; j < col.size (); j++)
        {
          PointT p = (PointT)inCloud (j, i);
          cloud->push_back (p);
        }
      }
      return cloud;
    }
}
#endif /* POINTCLOUD_UTILS_H_ */
