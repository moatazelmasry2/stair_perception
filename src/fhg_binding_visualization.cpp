/*
 * fhg_binding_filter.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: elmasry
 */

#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>

PCL_INSTANTIATE(PointCloudGeometryHandlerXYZ, (pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(PointCloudGeometryHandlerSurfaceNormal, (pcl::PointMoXYZRGBNormal))

