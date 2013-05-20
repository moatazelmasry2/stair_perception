#ifndef FHG_PCL_COMMONN_H_
#define FHG_PCL_COMMONN_H_

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include <pcl/common/transforms.h>

using namespace Eigen;

namespace pcl
{

  template<typename PointT>
  struct PointComparator
  {
    protected:
      int axis;
    public:
      PointComparator (int axis)
      {
        this->axis = axis;
      }

      bool operator() (const PointT& p1, const PointT& p2)
      {
        switch (axis)
        {
          case 0:
            return p1.x < p2.x;
          case 1:
            return p1.y < p2.y;
          case 2:
            return p1.z < p2.z;
          default:
            throw std::runtime_error ("pcl::PointComparator dimension must be between 0-2");
        }

      }
  };

  template<typename PointT>
  inline PointT&
  copyPoint (PointT& input);

  template<typename PointT>
  inline PointT vectorToPoint (Eigen::Vector3f input)
  {
    PointT p;
    p.x = input[0];
    p.y = input[1];
    p.z = input[2];
    return p;
  }

  template<typename PointT>
  inline const Eigen::Vector3d&
  pointToVec3d (PointT input)
  {
    Eigen::Vector3f pVec = input.getVector3fMap ();
    const Eigen::Vector3d& t = pVec.cast<double> ();
    //const Eigen::Vector3d& t = input.getVector3dMap ();
    return t;
  }

  inline double calcAbsoluteAngle (Eigen::Vector3d vec1, Eigen::Vector3d vec2)
  {
    double angle = acos ( (vec1.dot (vec2)) / (vec1.norm () * vec2.norm ())) * 180 / M_PI;
    angle = fabs (angle);
    int angleInt = (int) angle;
    double fraction = angle - angleInt;
    angleInt = angleInt % 180;
    return fabs (angleInt + fraction);
  }

  inline double calcAbsoluteAngle (Eigen::Vector3f vec1, Eigen::Vector3f vec2)
  {
    double angle = acos ( (vec1.dot (vec2)) / (vec1.norm () * vec2.norm ())) * 180 / M_PI;
    angle = fabs (angle);
    int angleInt = (int) angle;
    double fraction = angle - angleInt;
    angleInt = angleInt % 180;
    return fabs (angleInt + fraction);
  }

  /**
   * returns angle in degrees
   */
  inline double calcAnglebetweenVectors (Eigen::Vector3d vec1, Eigen::Vector3d vec2)
  {
    return acos ( (vec1.dot (vec2)) / (vec1.norm () * vec2.norm ())) * 180 / M_PI;
  }

  /**
   * returns angle in degrees
   */
  inline double calcAnglebetweenVectors (Eigen::Vector3f vec1, Eigen::Vector3f vec2)
  {
    return acos ( (vec1.dot (vec2)) / (vec1.norm () * vec2.norm ())) * 180 / M_PI;
  }

  template<typename PointT>
  bool isNanPoint (const PointT& point)
  {
    if (isnan (point.x || isnan (point.y) || isnan (point.z)))
    {
      return true;
    }
    return false;
  }

  template<typename PointT>
  bool isZeroPoint (const PointT& point)
  {
    if (point.x == 0 && point.y == 0 && point.z == 0)
    {
      return true;
    }
    return false;
  }

  template<typename PointT>
  inline std::vector<PointT, Eigen::aligned_allocator<PointT> > sortPoints (const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points, int axis)
  {
    std::vector<PointT, Eigen::aligned_allocator<PointT> > outPoints;
    outPoints.assign(points.begin(), points.end());
    PointComparator<PointT> comp (axis);
    std::sort (outPoints.begin (), outPoints.end (), comp);
    return outPoints;
  }

  template<typename PointT>
  inline bool samePoint (const PointT& p1, const PointT& p2)
  {
    return (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z);
  }

  template<typename PointT>
  inline PointT initZeroPoint ()
  {
    PointT p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    return p;
  }

  template<typename PointT>
  inline PointT initNaNPoint ()
  {
    PointT p;
    p.x = NAN;
    p.y = NAN;
    p.z = NAN;
    return p;
  }

  template<typename PointT>
  inline PointT rotatePoint (const PointT& point, Eigen::Quaternion<float> q)
  {
    Eigen::Vector3f vec = point.getVector3fMap ();
    vec = q * vec;
    return vectorToPoint<PointT> (vec);
  }

  template<typename PointT>
  inline PointT setPointNormal (const PointT& point, Eigen::Vector3f normal)
  {
    PointT p = point;
    p.normal_x = normal[0];
    p.normal_y = normal[1];
    p.normal_z = normal[2];
    return p;
  }

  template<typename PointT>
  PointT sumPoints (const PointT p1, const PointT p2)
  {
    PointT out;
    out.x = p1.x + p2.x;
    out.y = p1.y + p2.y;
    out.z = p1.z + p2.z;
    return out;
  }

  template<typename PointT>
  PointT subtractPoints (const PointT p1, const PointT p2)
  {
    PointT out;
    out.x = p1.x - p2.x;
    out.y = p1.y - p2.y;
    out.z = p1.z - p2.z;
    return out;
  }
}

//#include "impl/point_common.hpp"

#endif
