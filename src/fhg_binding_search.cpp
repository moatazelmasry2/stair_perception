/*
 * fhg_binding_search.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: elmasry
 */
#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

PCL_INSTANTIATE(Search, PCL_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(OrganizedNeighbor, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(KdTree, PCL_POINT_TYPES PointMoTypes)

//PCL_INSTANTIATE(Search, PCL_XYZ_POINT_TYPES PointMoTypes)
//PCL_INSTANTIATE(KdTree, PCL_XYZ_POINT_TYPES PointMoTypes)
//PCL_INSTANTIATE(OrganizedNeighbor, PCL_XYZ_POINT_TYPES PointMoTypes)


