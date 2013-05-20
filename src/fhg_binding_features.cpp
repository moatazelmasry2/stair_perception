/*
 * fhg_common_feature.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: elmasry
 */

#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/impl/integral_image_normal.hpp>
#include <pcl/features/impl/normal_3d.hpp>

PCL_INSTANTIATE_PRODUCT(IntegralImageNormalEstimation, ((pcl::PointXYZ)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointXYZRGBNormal) PointMoTypes)((pcl::Normal)(pcl::PointXYZRGBNormal)(pcl::PointMoXYZRGBNormal)))
PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointSurfel)(pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointNormal)PointMoTypes)((pcl::Normal)(pcl::PointNormal)(pcl::PointXYZRGBNormal)(pcl::PointMoXYZRGBNormal)))
