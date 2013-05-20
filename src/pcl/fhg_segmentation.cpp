/*
 * fhg_segmentation.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: elmasry
 */


#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>


#include "pcl/segmentation/region_segmentation.h"
#include "pcl/segmentation/planesegmentation.h"

//PCL_INSTANTIATE(PlaneSegmentation, (pcl::PointXYZRGBNormal)(pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE_PRODUCT(PlaneSegmentation, (PointMoTypes) ((pcl::PointMoXYZRGBNormal)) )
//PCL_INSTANTIATE(CurvatureEstimation, (pcl::PointXYZRGBNormal)(pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(PlanePolygonalization, (pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(InitialNormalRegionSegmentation, (pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(LastNormalRegionSegmentation, (pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(AverageNormalRegionSegmentation, (pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(ApproximatePlaneRegionSegmentation, (pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(ProbabilisticPlaneRegionSegmentation, (pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(PCLEuclideanRegionSegmentation, (pcl::PointMoXYZRGBNormal))
//PCL_INSTANTIATE(PCLNormalRegionSegmentation, (pcl::PointMoXYZRGBNormal))
