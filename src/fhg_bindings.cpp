/*
 * fhg_bindings.cpp
 *
 *  Created on: Feb 13, 2013
 *      Author: elmasry
 */




#include "pcl/common/fhg_point_types.h"

#include "pcl/pcl_base.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/impl/extract_indices.hpp"

#include "pcl/io/mo_io.h"

#define MOPOINTTYPES \
  (pcl::PointMoXYZRGB)\
  (pcl::PointMoXYZRGBNormal)

PCL_INSTANTIATE(ExtractIndices, MOPOINTTYPES )
