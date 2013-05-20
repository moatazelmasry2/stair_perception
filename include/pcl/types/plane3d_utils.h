/*
 * plane3d_utils.h
 *
 *  Created on: Jun 24, 2012
 *      Author: elmasry
 */

#ifndef PLANE3D_UTILS_H_
#define PLANE3D_UTILS_H_

#include <stdexcept>
#include <list>
#include <vector>
#include <Eigen/Dense>

#include "pcl/common/point_common.h"
#include "pcl/common/math.h"
#include "pcl/types/plane3d.h"
#include "pcl/types/plane3d_utils.h"
#include <pcl/registration/transforms.h>

#include "pcl/histogram/planehistogram.h"

using namespace Eigen;

namespace pcl
{
  namespace utils
  {
    /**
     * rotates camera coordinate to world coordinate
     */
    template<typename PointT>
    void cameraToworld (Plane3D<PointT>& plane)
    {
      Quaternion<float> q;
      q = Quaternion<float> (-0.500398163355, 0.499999841466, -0.499601836645, 0.499999841466);
      typename pcl::PointCloud<PointT>::ConstPtr cloud = plane.getCloud ();
      typename pcl::PointCloud<PointT>::Ptr cloudOut (new pcl::PointCloud<PointT>);
      pcl::transformPointCloud (*cloud, *cloudOut, Eigen::Vector3f (0, 0, 0), q);
      plane.setInputCloud (cloudOut);
    }

    /**
     * rotates camera coordinate to world coordinate
     */
    template<typename PointT>
    void cameraToworld (std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> >& planes)
    {
      for (size_t i = 0; i < planes.size (); i++)
      {
        pcl::Plane3D<PointT>& plane = planes[i];
        cameraToworld (plane);
      }
    }

    /**
     *
     */
    template<typename PointT>
    void rotatePlane (Plane3D<PointT>& plane, double angle, Eigen::Vector3f axis)
    {
      Quaternion<float> q;
      q = AngleAxis<float> (angle * M_PI / 180, axis);
      typename pcl::PointCloud<PointT>::ConstPtr cloud = plane.getCloud ();
      typename pcl::PointCloud<PointT>::Ptr cloudOut (new pcl::PointCloud<PointT>);
      pcl::transformPointCloud (*cloud, *cloudOut, Eigen::Vector3f (0, 0, 0), q);
      //plane.normal = q * plane.normal;
      //plane.center = q * plane.center;
      plane.setInputCloud (cloudOut);
    }

    /**
     *
     */
    template<typename PointT>
    void rotatePlanes (std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> >& planes, double angle,
        Eigen::Vector3f axis)
    {
      for (size_t i = 0; i < planes.size (); i++)
      {
        pcl::Plane3D<PointT>& plane = planes[i];
        rotatePlane (plane, angle, axis);
      }
    }

    template<typename PointT>
    void logPlanes (const std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> >& planes)
    {
      for (size_t i = 0; i < planes.size (); i++)
      {
        const pcl::Plane3D<PointT>& plane = planes[i];
        plane.logPlane ();
      }
    }

    template<typename PointT>
    struct Plane3DComparator
    {
        int index;
        Plane3DComparator (int index)
        {
          this->index = index;
        }

        bool operator() (const pcl::Plane3D<PointT>& plane1, const pcl::Plane3D<PointT>& plane2)
        {
          if (index == 0)
          {
            return plane1.center[0] < plane2.center[0] ? true : false;
          }
          else if (index == 1)
          {
            return plane1.center[1] < plane2.center[1] ? true : false;
          }
          else if (index == 2)
          {
            return plane1.center[2] < plane2.center[2] ? true : false;
          }
          else
          {
            return false;
          }
        }
    };
    /**
     *  sort planes according to an index representing x,y,z
     */
    template<typename PointT>
    void sortPlanes (std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> >& planes, int index)
    {

      std::list<pcl::Plane3D<PointT>, Eigen::aligned_allocator<PointT> > planesList;
      planesList.assign (planes.begin (), planes.end ());
      planesList.sort (Plane3DComparator<PointT> (index));
      planes.clear ();
      planes.assign (planesList.begin (), planesList.end ());
    }

    template<typename PointT>
    Eigen::Vector3f calcAvgCenters (const std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> >& planes)
    {
      Eigen::Vector3f avg (0, 0, 0);
      for (size_t i = 0; i < planes.size (); i++)
      {
        avg += planes[i].getCenter ();
      }
      return avg / (int) planes.size ();
    }

    /**
     * removes planes smaller than certain LxWxH and ignores non Tread, non Riser
     */
    template<typename PointT>
    std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> > const removeSmallPlanes (
        const std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> >& planes, float minLength, float minDepth,
        float minHeight)
    {
      std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> > outPlanes;
      for (size_t i = 0; i < planes.size (); i++)
      {
        const Plane3D<PointT>& plane = planes[i];
        if (plane.isRiser ())
        {
          if (plane.height < minHeight || plane.length < minLength)
            continue;
          outPlanes.push_back (plane);
        }
        else if (plane.isTread ())
        {
          if (plane.depth < minDepth || plane.length < minLength)
            continue;
          outPlanes.push_back (plane);
        }
      }
      return outPlanes;
    }

    template<typename PointT>
    bool isWall (const Plane3D<PointT>& plane)
    {
      if (isVertical (plane.getNormal (), Eigen::Vector3f (1, 0, 0))
          && isHorizontal (plane.getNormal (), Eigen::Vector3f (0, 1, 0))
          && isVertical (plane.getNormal (), Eigen::Vector3f (0, 0, 1)))
        return true;
      return false;
    }

    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr sampleBoundingBox (
        std::vector<PointT, Eigen::aligned_allocator<PointT> > bbox)
    {
      typename pcl::PointCloud<PointT>::Ptr bordersCloud (new pcl::PointCloud<PointT>);
      typename pcl::PointCloud<PointT>::Ptr edge;
      for (size_t j = 0; j < bbox.size () - 1; j++)
      {
        edge = reconstructLineSegment (bbox[j], bbox[j + 1], 0.01f);
        concatePointClouds (*edge, *bordersCloud);
      }
      edge = reconstructLineSegment (bbox[3], bbox[0], 0.01f);
      concatePointClouds (*edge, *bordersCloud);
      return bordersCloud;
    }

    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr sampleBoundingBox (pcl::Plane3D<PointT> plane)
    {
      std::vector<PointT, Eigen::aligned_allocator<PointT> > bbox;
      bbox.push_back (plane.bbox.topLeft);
      bbox.push_back (plane.bbox.topRight);
      bbox.push_back (plane.bbox.bottomRight);
      bbox.push_back (plane.bbox.bottomLeft);
      return sampleBoundingBox<PointT> (bbox);
    }

    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr sampleBoundingBoxes (
        const std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> >& planes)
    {
      typename pcl::PointCloud<PointT>::Ptr bordersCloud (new pcl::PointCloud<PointT>);
      for (size_t i = 0; i < planes.size (); i++)
      {
        pcl::concatePointClouds (*sampleBoundingBox<PointT> (planes[i]), *bordersCloud);
      }
      return bordersCloud;
    }

    template<typename PointT>
    std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> > mergePlanes (
        std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> > planes, float stepSize = 0.07,
        float distanceThreshold = 0.1)
    {
      typedef std::vector<Plane3D<PointT>, Eigen::aligned_allocator<PointT> > Plane3DVector;
      pcl::PlaneHistogram<pcl::Plane3D<PointT>, PointT> horHist (2), vertHist (0);
      for (typename Plane3DVector::iterator it = planes.begin (); it != planes.end (); it++)
      {
        Plane3D<PointT>& plane = *it;
        if (plane.isTread ())
        {
          horHist.addPlane (plane);
        }
        else if (plane.isRiser ())
        {
          vertHist.addPlane (plane);
        }
      }

      horHist.setStepsize (stepSize);
      vertHist.setStepsize (stepSize);
      horHist.maxDistanceThreshold = distanceThreshold;
      vertHist.maxDistanceThreshold = distanceThreshold;
      horHist.compute ();
      vertHist.compute ();
      std::vector<Plane3D<PointT>, Eigen::aligned_allocator<Plane3D<PointT> > > hPlanes = horHist.getOutplanes (),
          vPlanes = vertHist.getOutplanes ();
      planes.clear ();
      planes.insert (planes.end (), hPlanes.begin (), hPlanes.end ());
      planes.insert (planes.end (), vPlanes.begin (), vPlanes.end ());
      return planes;
    }
  }
}

#endif /* PLANE3D_UTILS_H_ */
