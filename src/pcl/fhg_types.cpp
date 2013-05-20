/*
 * types.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: elmasry
 */

#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

#include "pcl/types/boundingbox.h"
#include "pcl/types/linesegment3d.h"
#include "pcl/types/plane3d_utils.h"
#include "pcl/types/plane3d.h"
#include "pcl/types/riser.h"
#include "pcl/types/step.h"
#include "pcl/types/tread.h"
#include "pcl/types/types.h"



PCL_INSTANTIATE(LineSegment3D, (pcl::PointMoXYZRGBNormal) )
PCL_INSTANTIATE(BoundingBox, (pcl::PointMoXYZRGBNormal) )
PCL_INSTANTIATE(Plane3D, (pcl::PointMoXYZRGBNormal) )
PCL_INSTANTIATE(Tread, (pcl::PointMoXYZRGBNormal) )
PCL_INSTANTIATE(Riser, (pcl::PointMoXYZRGBNormal) )
PCL_INSTANTIATE(Step, (pcl::PointMoXYZRGBNormal) )
