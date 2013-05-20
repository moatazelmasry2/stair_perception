/*
 * fhg_binding_surface.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: elmasry
 */
#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

#include <pcl/surface/impl/convex_hull.hpp>
#include <pcl/surface/impl/organized_fast_mesh.hpp>
#include <pcl/surface/impl/gp3.hpp>

PCL_INSTANTIATE(ConvexHull, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(OrganizedFastMesh, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(GreedyProjectionTriangulation, (pcl::PointNormal)(pcl::PointXYZRGBNormal)(pcl::PointXYZINormal)(pcl::PointMoXYZRGBNormal))



