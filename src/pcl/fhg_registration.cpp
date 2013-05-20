/*
 * fhg_registration.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: elmasry
 */


#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

#include "pcl/registration/icp_dirk.h"
#include "pcl/registration/registration_dirk.h"
#include "pcl/registration/myicp.h"

PCL_INSTANTIATE_PRODUCT(MyICP, ((pcl::PointXYZRGBNormal)(pcl::PointMoXYZRGBNormal)) ((pcl::PointXYZRGBNormal)(pcl::PointMoXYZRGBNormal)) )
PCL_INSTANTIATE_PRODUCT(MyICPNonLinear, ((pcl::PointXYZRGBNormal)(pcl::PointMoXYZRGBNormal)) ((pcl::PointXYZRGBNormal)(pcl::PointMoXYZRGBNormal)) )
