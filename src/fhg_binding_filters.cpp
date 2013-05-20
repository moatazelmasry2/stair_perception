/*
 * fhg_binding_filter.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: elmasry
 */

#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

PCL_INSTANTIATE(ExtractIndices, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointXYZRGBNormal) PointMoTypes)
PCL_INSTANTIATE(getMinMax3D, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(VoxelGrid, PCL_XYZ_POINT_TYPES PointMoTypes)
