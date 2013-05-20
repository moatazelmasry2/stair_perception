/*
 * model_utils.h
 *
 *  Created on: Oct 2, 2012
 *      Author: elmasry
 */

#ifndef MODEL_UTILS_H_
#define MODEL_UTILS_H_

#include "pcl/types/plane3d.h"
#include "pcl/types/linesegment3d.h"
#include "pcl/common/linesegment3d_common.h"

namespace pcl
{
  namespace model
  {

    template<typename PointT>
    struct LinesComparator
    {
      bool
      operator() (LineSegment3D<PointT> line1, LineSegment3D<PointT> line2)
      {
        return line1.getCenter ()[1] < line2.getCenter ()[1];
      }
    };

    /**
     * Takes a group of vertical lines, and tries to find Trail/walls from them
     */
    template<typename PointT>
      std::vector<pcl::Plane3D<PointT>, Eigen::aligned_allocator<PointT> >
      lines2Trails (const std::vector<pcl::LineSegment3D<PointT>, Eigen::aligned_allocator<PointT> >& inlines)
      {

        float maxRotationDeviationBetLines = 15;
        float minDistBetLines = 0.05;
        float maxDistLinesOnSamePlane = 0.04;
        std::vector<pcl::Plane3D<PointT>, Eigen::aligned_allocator<PointT> > walls;

        std::vector<Plane3D<PointT> , Eigen::aligned_allocator<PointT> > planes;
        //LinesComparator<PointT> comp;
        //TODO fix this line
        //std::sort (inlines.begin (), inlines.end (), comp);
        std::vector<int> linesLedger (inlines.size (), 2);

        int linesCounter =  0;
        for (size_t i = 0; i < inlines.size (); i++)
        {
          if (linesLedger[i] <= 0) continue;
          const LineSegment3D<PointT>& line1 = inlines[i];
          for (size_t j = i + 1; j <= i + 3; j++)
          {
            if (linesLedger[j] <= 0) continue;
            if (j >= inlines.size ()) break;
            const LineSegment3D<PointT>& line2 = inlines[j];
            if (line1.getId () == line2.getId ()) continue;
            if (::pcl::areParallel (line1, line2, maxRotationDeviationBetLines))
            {
              PointT topLine1, bottomLine1, topLine2, bottomLine2;
              if (line1.getLineModel  ()[0].z > line1.getLineModel  ()[1].z)
              {
                topLine1 = line1.getLineModel  ()[0];
                bottomLine1 = line1.getLineModel  ()[1];
              } else
              {
                topLine1 = line1.getLineModel  ()[1];
                bottomLine1 = line1.getLineModel  ()[0];
              }

              if (line2.getLineModel  ()[0].z > line2.getLineModel  ()[1].z)
              {
                topLine2 = line2.getLineModel  ()[0];
                bottomLine2 = line2.getLineModel  ()[1];
              } else
              {
                topLine2 = line2.getLineModel  ()[1];
                bottomLine2 = line2.getLineModel  ()[0];
              }

              if (topLine1.z < bottomLine2.z || bottomLine1.z > topLine2.z) continue;

              if ((fabs (line1.getCenter ()[1] - line2.getCenter ()[1]) < maxDistLinesOnSamePlane
                  && fabs (line1.getCenter ()[0] - line2.getCenter ()[0]) > minDistBetLines))
              {
                Plane3D<PointT> plane (linesCounter++);
                plane.setModelWall (line1, line2);

                walls.push_back (plane);
                //plane.logPlane();
                planes.push_back (plane);
                linesLedger[i]--;
                linesLedger[j]--;
              }//end if
            }
          }
        }
        return walls;
      }

    template<typename PointT>
      std::vector<pcl::Plane3D<PointT>, Eigen::aligned_allocator<PointT> >
      buildWalls (const std::vector<pcl::LineSegment3D<PointT>, Eigen::aligned_allocator<PointT> >& inlines)
      {
        std::vector<pcl::Plane3D<PointT>, Eigen::aligned_allocator<PointT> > outWalls;

      }
  }
}
#endif /* MODEL_UTILS_H_ */
