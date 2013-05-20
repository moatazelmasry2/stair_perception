/*
 * fhg_histogram.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: elmasry
 */

#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

#include "pcl/apps/fast_meshing.h"
#include "pcl/apps/stairdetectiondemo.h"

PCL_INSTANTIATE_PRODUCT(FastMeshing, (PointMoTypes) ((pcl::PointMoXYZRGBNormal)) )
PCL_INSTANTIATE_PRODUCT(StairDetectionDemo, (PointMoTypes) ((pcl::PointMoXYZRGBNormal)) )


