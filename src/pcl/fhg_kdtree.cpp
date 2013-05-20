/*
 * fhg_kdtree.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: elmasry
 */

//#include "pcl/kdtree/fixed_neighbors.h"
//#include "pcl/kdtree/impl/fixed_neighbors.hpp"

#include "pcl/impl/instantiate.hpp"
#include "pcl/point_types.h"
#include "pcl/kdtree/fixed_neighbors.h"
#include "pcl/kdtree/impl/fixed_neighbors.hpp"

// Instantiations of specific point types
PCL_INSTANTIATE(FixedNeighbors, PCL_XYZ_POINT_TYPES);
