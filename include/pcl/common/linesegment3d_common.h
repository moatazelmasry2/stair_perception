/*
 * linesegment3d_common.h
 *
 *  Created on: Aug 31, 2012
 *      Author: elmasry
 */

#ifndef LINESEGMENT3D_COMMON_H_
#define LINESEGMENT3D_COMMON_H_

#include "pcl/common/math.h"
#include "pcl/types/linesegment3d.h"

namespace pcl
{

  /**
   * \param toleranceAngle how much rotation is tolerated between lines to still be considered parallel, in degrees
   */
  template<typename PointT>
    inline bool
    areParallel (const LineSegment3D<PointT>& line1, const LineSegment3D<PointT>& line2, float toleranceAngle = 10)
    {
      return areParallel (line1.getNormal (), line2.getNormal (), toleranceAngle);
    }

  template<typename PointT>
  void
  logLines (const std::vector<LineSegment3D<PointT> , Eigen::aligned_allocator<PointT> >& lines)
  {
    printf ("numLines=%d\n", (int)lines.size ());
    for (size_t i = 0; i < lines.size (); i++)
    {
      const LineSegment3D<PointT>& line = lines[i];
      line.logLine();
    }
  }
}
#endif /* LINESEGMENT3D_COMMON_H_ */
