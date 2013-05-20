/*
 * fhg_binding_segmentation.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: elmasry
 */

#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"

#include <pcl/common/fhg_point_types.h>

#include <pcl/segmentation/impl/sac_segmentation.hpp>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/extract_labeled_clusters.h>
#include <pcl/segmentation/impl/extract_labeled_clusters.hpp>

PCL_INSTANTIATE(SACSegmentation, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)PointMoTypes)

// Instantiations of specific point types
//PCL_INSTANTIATE(EuclideanClusterExtraction, PCL_XYZ_POINT_TYPES PointMoTypes)
//PCL_INSTANTIATE(extractEuclideanClusters, PCL_XYZ_POINT_TYPES PointMoTypes)
//PCL_INSTANTIATE(extractEuclideanClusters_indices, PCL_XYZ_POINT_TYPES PointMoTypes)
//PCL_INSTANTIATE(extractLabeledEuclideanClusters, PCL_XYZL_POINT_TYPES PointMoTypes)
