/*
 * commons.h
 *
 *  Created on: Mar 13, 2012
 *      Author: elmasry
 */

#ifndef COMMONS_H_
#define COMMONS_H_

#include <vector>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>

namespace pcl
{
  template<typename PointT>
    inline double
    calcDistance (PointT p1, PointT p2)
    {
      return sqrt (pow ((p1.x - p2.x), 2) + pow ((p1.y - p2.y), 2) + pow ((p1.z - p2.z), 2));
    }

//  template<typename PointT>
//    inline void
//    reconstructLineSegment (PointT extremeLeft, PointT extremeRight,
//                            std::vector<PointT, Eigen::aligned_allocator<PointT> >& out)
//    {
//      double dist = calcDistance (extremeLeft, extremeRight);
//      int numPoints = (int)dist / 2;
//      //reconstruct points along the edge with ratio 2mm : 1point
//      /*double xstep = fabs (fabs (extremeLeft.x) - fabs (extremeRight.x)) / numPoints;
//       double ystep = fabs (fabs (extremeLeft.y) - fabs (extremeRight.y)) / numPoints;
//       double zstep = fabs (fabs (extremeLeft.z) - fabs (extremeRight.z)) / numPoints;*/
//      double xstep = (extremeLeft.x - extremeRight.x) / numPoints;
//      double ystep = (extremeLeft.y - extremeRight.y) / numPoints;
//      double zstep = (extremeLeft.z - extremeRight.z) / numPoints;
//
//      int xsign = 1, ysign = 1, zsign = 1;
//      if (extremeLeft.x > extremeRight.x)
//      {
//        xsign = -1;
//      }
//      if (extremeLeft.y > extremeRight.y)
//      {
//        ysign = -1;
//      }
//      if (extremeLeft.z > extremeRight.z)
//      {
//        zsign = -1;
//      }
//
//      PointT tmp = extremeLeft;
//
//      out.push_back (extremeLeft);
//      //for (int i = 0; i < numPoints; i++)
//      //while (calcDistance(tmp, extremeRight) > 20)
//      //std::cout << "p1=" << extremeLeft << ", p2=" << extremeRight << ", numPoints=" << numPoints << ", xstep="
//      //    << xstep;
//      //std::cout << ", ystep=" << ystep << "zstep=" << zstep << std::endl;
//
//      while (fabs (tmp.y - extremeRight.y) > 10)
//      {
//        if (fabs (tmp.x - extremeRight.x) > 10)
//        {
//          tmp.x -= xstep;
//        }
//        if (fabs (tmp.y - extremeRight.y) > 10)
//        {
//          tmp.y -= ystep;
//        }
//        if (fabs (tmp.z - extremeRight.z) > 10)
//        {
//          tmp.z -= zstep;
//        }
//
//        out.push_back (tmp);
//      }
//      out.push_back (extremeRight);
//    }

  /*inline long
  getTimeMs ()
  {
    struct timeval start;
    long mtime;

    gettimeofday (&start, NULL);

    mtime = ((start.tv_sec) * 1000 + start.tv_usec / 1000.0) + 0.5;

    return mtime;
  }*/
}

#endif /* COMMONS_H_ */
