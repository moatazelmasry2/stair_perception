/*
 * stairdetector.h
 *
 *  Created on: Jul 26, 2012
 *      Author: elmasry
 */

#ifndef STAIRDETECTOR_H_
#define STAIRDETECTOR_H_

#include <vector>
#include <algorithm>

#include "pcl/types/plane3d.h"
#include "pcl/types/step.h"
#include "pcl/common/color.h"
#include "pcl/utils/pointcloud_utils.h"
#include "pcl/types/plane3d_utils.h"
#include "pcl/types/tread.h"
#include "pcl/types/riser.h"

namespace pcl
{
  template<typename PointT>
    class StairDetector
    {

      static const int compIndex = 2;
      //camera coordinate system values
      static const float maxXDist = 0.8;
      static const float maxYDist = 0.4;
      static const float maxZDist = 0.4;

      typedef Eigen::aligned_allocator<PointT> Alloc;
      typedef std::vector<Plane3D<PointT> , Alloc> Plane3DVector;
      typedef std::vector<Step<PointT> , Alloc> StepsVector;
      Plane3DVector planes;
      StepsVector steps;

      struct Comparator
      {
        int compIndex;
        Comparator (int compIndex)
        {
          this->compIndex = compIndex;
        }
        bool
        operator() (const pcl::Plane3D<PointT>& plane1, const pcl::Plane3D<PointT>& plane2)
        {
          return plane1.center[compIndex] < plane2.center[compIndex] ? true : false;
        }
      };

    protected:

      void
      computeSteps (Plane3DVector& planes, StepsVector& steps, int compIndex = 2)
      {
        steps.clear();
        Eigen::Vector3f avgCenter = pcl::utils::calcAvgCenters(planes);
        float xDifferenceThreshold = 0.8;
        Plane3DVector tmpPlanes;

        for (size_t i = 0; i < planes.size(); i++) {
          if (fabs(planes[i].center[1] - avgCenter[1]) < xDifferenceThreshold ) {
            tmpPlanes.push_back(planes[i]);
          }
        }
        planes.clear();
        planes.assign(tmpPlanes.begin(), tmpPlanes.end());


        for (size_t i = 0; i < planes.size (); i++)
        {
          bool added = false;
          Plane3D<PointT>& plane = planes[i];

          if (plane.isRiser ())
          {
            typename Riser<PointT>::Ptr riser = boost::static_pointer_cast<Riser<PointT> >(plane.makeShared());
            //Riser<PointT> riser;
            for (size_t j = 0; j < steps.size (); j++)
            {
              Step<PointT>& step = steps[j];
              if (!step.hasRiser ())
              {
                const Tread<PointT>& tread = step.getTread ();
                float dist = tread.center[0] - plane.center[0];
                if (fabs (tread.center[1] - plane.center[1]) < maxXDist && dist
                    < maxZDist && dist > 0 && fabs (tread.center[2] - plane.center[2]) < maxYDist)
                {
                  added = true;

                  step.setRiser (*riser);
                }
              }
            }
            if (!added)
            {
              Step<PointT> step;

              step.setRiser (*riser);
              steps.push_back (step);
            }
          }
          else if (plane.isTread ())
          {
            //Tread<PointT> tread;
            typename Tread<PointT>::Ptr tread = boost::static_pointer_cast<Tread<PointT> >(plane.makeShared());
            for (size_t j = 0; j < steps.size (); j++)
            {
              Step<PointT>& step = steps[j];
              if (!step.hasTread ())
              {
                const Plane3D<PointT>& riser = step.getRiser ();
                float dist = plane.center[0] - riser.center[0];
                if (fabs (riser.center[1] - plane.center[1]) < maxXDist && dist
                    < maxZDist && dist > 0 && fabs (riser.center[2] - plane.center[2]) < maxYDist)
                {
                  added = true;
                  step.setTread (*tread);
                }
              }
            }
            if (!added)
            {
              Step<PointT> step;
              step.setTread(*tread);
              steps.push_back (step);
            }
          }
        }
      }
    public:

      void
      addPlanes (Plane3DVector& inPlanes)
      {
        planes.insert (planes.end (), inPlanes.begin (), inPlanes.end ());
        Plane3DVector vec;

      }

      void
      addPlane (Plane3D<PointT>& inPlane)
      {
        planes.push_back (inPlane);
      }

      void
      compute ()
      {
        Comparator comp (compIndex);
        std::sort (planes.begin (), planes.end (), comp);
        computeSteps (planes, steps);
      }

      inline StepsVector
      getSteps ()
      {
        return steps;
      }

      /**
       * get the detected steps as colored cloud, where each step has a different color
       */
      typename pcl::PointCloud<PointT>::Ptr
      getColoredCloud ()
      {
        //printf ("number of steps=%d\n", (int)steps.size ());

        typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        for (size_t i = 0; i < steps.size (); i++)
        {
          Step<PointT>& step = steps[i];

          float color = pcl::color::getRandomColor ();
          if (step.hasRiser ())
          {
            typename pcl::PointCloud<PointT>::Ptr tmpCloud (new pcl::PointCloud<PointT>);
            tmpCloud = step.getRiser ().getCloud ();
            pcl::colorCloud<PointT> (tmpCloud, color);
            pcl::concatePointClouds (*tmpCloud, *cloud);
          }
          if (step.hasTread ())
          {
            typename pcl::PointCloud<PointT>::Ptr tmpCloud (new pcl::PointCloud<PointT>);
            tmpCloud = step.getTread ().getCloud ();
            pcl::colorCloud<PointT> (tmpCloud, color);
            pcl::concatePointClouds (*tmpCloud, *cloud);
          }
        }
        return cloud;
      }

    };
}
#endif /* STAIRDETECTOR_H_ */
