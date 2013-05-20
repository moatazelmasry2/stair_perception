/*
 * fhg_io.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: elmasry
 */

#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

//#include "io/mo_io.cpp"
#include "pcl/io/mo_io.h"
#include "pcl/io/globfitwriter.h"

PCL_INSTANTIATE(GlobfitWriter,  (pcl::PointMoXYZRGBNormal) )
