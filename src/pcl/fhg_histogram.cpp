/*
 * fhg_histogram.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: elmasry
 */

#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

#include "pcl/histogram/linesegment3dhistogram.h"
#include "pcl/histogram/linesegmenthistogram.h"
#include "pcl/histogram/planehistogram.h"


PCL_INSTANTIATE(LineSegment3DHistogram, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(LineSegmentBin, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(LineSegmentHistogram, PCL_XYZ_POINT_TYPES PointMoTypes)
