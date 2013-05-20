/*
 * edgeslocalmodelfactory.h
 *
 *  Created on: Aug 31, 2012
 *      Author: elmasry
 */

#ifndef EDGES2PLANESFACTORY_H_
#define EDGES2PLANESFACTORY_H_

#include "pcl/common/pcl_commons.h"
#include "pcl/types/linesegment3d.h"
#include "pcl/common/linesegment3d_common.h"
#include "pcl/types/plane3d.h"
#include "pcl/types/plane3d_utils.h"
#include "pcl/io/mo_io.h"

namespace pcl
{
  template<typename PointT>
    class Edges2PlanesFactory
    {

      const static float maxRotationDeviationBetLines = 15;
      const static float minDistBetLines = 0.05;
      const static float maxDistLinesOnSamePlane = 0.04;

      int counter;

    protected:
      std::vector<LineSegment3D<PointT> , Eigen::aligned_allocator<PointT> > lines;

    public:

      Edges2PlanesFactory() {
        counter = 0;
      }
      inline void
      setEdges (const std::vector<LineSegment3D<PointT> , Eigen::aligned_allocator<PointT> >& lines)
      {
        this->lines = lines;
        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        counter = 0;
        for (size_t i = 0; i < this->lines.size(); i++) {
          LineSegment3D<PointT>& line = this->lines[i];
          line.setId(counter++);
          pcl::concatePointClouds(*line.getCloud(), *cloud);
        }

        pcl::io::saveMoPcd("lines_cloud.pcd", *cloud);

        //::pcl::logLines(this->lines);
      }

      /**
       * compares neighboring lines and create planes.
       */
      std::vector<Plane3D<PointT> , Eigen::aligned_allocator<PointT> >
      createPlanes ()
      {
        std::sort (lines.begin (), lines.end ());
//        logLines (lines);
        std::vector<Plane3D<PointT> , Eigen::aligned_allocator<PointT> > planes;
        std::vector<int> linesLedger (lines.size (), 2);
        int count = 0;
        //printf ("create planes from edges. num edges=%d\n", (int)lines.size ());
        for (size_t i = 0; i < lines.size (); i++)
        {
          if (linesLedger[i] <= 0) continue;
          LineSegment3D<PointT>& line1 = lines[i];
          for (size_t j = i + 1; j <= i + 3; j++)
          {
            if (linesLedger[j] <= 0) continue;
            if (j >= lines.size ()) break;
            LineSegment3D<PointT>& line2 = lines[j];
            if (line1.getId () == line2.getId ()) continue;
            //printf ("compare lines %d, %d, ", line1.getId (), line2.getId ());
            if (areParallel (line1, line2, 80.0f) )
            {
              //printf (", areparallel");
//              printf ("lines %d, %d are parallel, ", line1.getId (), line2.getId ());
              PointT leftLine1, rightLine1, leftLine2, rightLine2;
              if (line1.getLineModel  ()[0].y > line1.getLineModel  ()[1].y)
              {
                leftLine1 = line1.getLineModel  ()[0];
                rightLine1 = line1.getLineModel  ()[1];
              } else
              {
                leftLine1 = line1.getLineModel  ()[1];
                rightLine1 = line1.getLineModel  ()[0];
              }

              if (line2.getLineModel  ()[0].y > line2.getLineModel  ()[1].y)
              {
                leftLine2 = line2.getLineModel  ()[0];
                rightLine2 = line2.getLineModel  ()[1];
              } else
              {
                leftLine2 = line2.getLineModel  ()[1];
                rightLine2 = line2.getLineModel  ()[0];
              }

              if (leftLine1.y < rightLine2.y || rightLine1.y > leftLine2.y) continue;
              //printf (", same y");
              if ((fabs (line1.getCenter ()[0] - line2.getCenter ()[0]) < maxDistLinesOnSamePlane
                  && fabs (line1.getCenter ()[2] - line2.getCenter ()[2]) > minDistBetLines)
                  || (fabs (line1.getCenter ()[2] - line2.getCenter ()[2]) < maxDistLinesOnSamePlane
                      && fabs (line1.getCenter ()[0] - line2.getCenter ()[0]) > minDistBetLines))
              {
                Plane3D<PointT> plane (count++);
                plane.setPlaneModel (line1, line2);

                if (plane.isTread () || plane.isRiser ())
                {
                  //plane.logPlane();
                  planes.push_back (plane);
                  linesLedger[i]--;
                  linesLedger[j]--;
                }
              }
            }//end if parallel
            //printf ("\n");fflush(stdout);
          }//end inner for loop
        }//end outer for loop
        //printf ("created %d planes\n", (int)planes.size ());
        //logPlanes (planes);
        return pcl::utils::mergePlanes(planes);
      }
    };
}

#define PCL_INSTANTIATE_Edges2PlanesFactory(T) template class PCL_EXPORTS pcl::Edges2PlanesFactory<T>;

#endif /* EDGESLOCALMODELFACTORY_H_ */
