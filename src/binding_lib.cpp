/*
 * binding_lib.cpp
 *
 *  Created on: Feb 13, 2013
 *      Author: elmasry
 */
#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"

#include <pcl/common/fhg_point_types.h>



#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>


#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/impl/integral_image_normal.hpp>
#include <pcl/features/impl/normal_3d.hpp>

#include <pcl/kdtree/impl/fixed_neighbors.hpp>

#include <pcl/segmentation/impl/sac_segmentation.hpp>

#include <pcl/filters/impl/extract_indices.hpp>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/impl/correspondence_estimation.hpp>

#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/impl/kdtree.hpp>

//#include <pcl/surface/impl/reconstruction.hpp>
#include <pcl/surface/impl/convex_hull.hpp>
#include <pcl/surface/impl/organized_fast_mesh.hpp>
#include <pcl/surface/impl/gp3.hpp>

#include <pcl/sample_consensus/impl/lmeds.hpp>
#include <pcl/sample_consensus/impl/msac.hpp>
#include <pcl/sample_consensus/impl/rmsac.hpp>
#include <pcl/sample_consensus/impl/rransac.hpp>
#include <pcl/sample_consensus/impl/mlesac.hpp>
#include <pcl/sample_consensus/impl/prosac.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_circle.hpp>
#include <pcl/sample_consensus/impl/sac_model_circle3d.hpp>
#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_stick.hpp>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/impl/rransac.hpp>

#include <pcl/segmentation/impl/sac_segmentation.hpp>



PCL_INSTANTIATE(PCLBase, PCL_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(ExtractIndices, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointXYZRGBNormal) PointMoTypes)
PCL_INSTANTIATE_PRODUCT(IntegralImageNormalEstimation, ((pcl::PointXYZ)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointXYZRGBNormal) PointMoTypes)((pcl::Normal)(pcl::PointXYZRGBNormal)(PointMoXYZRGBNormal)))
PCL_INSTANTIATE(Search, PCL_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(OrganizedNeighbor, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(KdTree, PCL_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(SACSegmentation, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)PointMoTypes)
PCL_INSTANTIATE(ConvexHull, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(OrganizedFastMesh, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE_PRODUCT(NormalEstimation, ((pcl::PointSurfel)(pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointNormal)PointMoTypes)((pcl::Normal)(pcl::PointNormal)(pcl::PointXYZRGBNormal)(PointMoXYZRGBNormal)))
PCL_INSTANTIATE(GreedyProjectionTriangulation, (pcl::PointNormal)(pcl::PointXYZRGBNormal)(pcl::PointXYZINormal)(PointMoXYZRGBNormal))
PCL_INSTANTIATE(FixedNeighbors, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(SampleConsensusModelCircle2D, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(SampleConsensusModelCircle3D, PCL_XYZ_POINT_TYPES PointMoTypes)

