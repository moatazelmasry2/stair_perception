/*
 * fhg_binding_kdtree.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: elmasry
 */

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/common/fhg_point_types.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/kdtree/fixed_neighbors.h>
#include <pcl/kdtree/impl/fixed_neighbors.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

PCL_INSTANTIATE(KdTreeFLANN, PCL_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(FixedNeighbors, PCL_XYZ_POINT_TYPES PointMoTypes)
