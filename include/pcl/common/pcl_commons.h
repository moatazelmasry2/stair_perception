/*
 * pcl_commons.h
 *
 *  Created on: May 23, 2012
 *      Author: elmasry
 */

#ifndef PCL_COMMONS_H_
#define PCL_COMMONS_H_

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "pcl/sample_consensus/method_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/sac_model_line.h"

#include <pcl/PolygonMesh.h>
#include <pcl/common/angles.h>
#include <pcl/common/mesh_utilities.h>
#include <pcl/surface/surface_functions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>


#include "pcl/registration/correspondence_estimation.h"
#include "pcl/registration/correspondence_rejection_distance.h"
#include "pcl/registration/correspondence_rejection_trimmed.h"

#include "pcl/registration/correspondence_rejection_sample_consensus.h"
#include "pcl/registration/transformation_estimation_svd.h"

#include "pcl/correspondence.h"
#include "pcl/registration/correspondence_rejection_one_to_one.h"


//#include <pcl/registration/correspondence_rejection_one_to_one.h>

//#include "pcl/utils/pointcloud_utils.h"

namespace pcl
{
  float
  generateColor (int r, int g, int b)
  {
    uint8_t _r = (uint8_t)r;
    uint8_t _g = (uint8_t)g;
    uint8_t _b = (uint8_t)b;
    uint32_t rgb = ((uint32_t)_r << 16 | (uint32_t)_g << 8 | (uint32_t)_b);
    return *reinterpret_cast<float*> (&rgb);
  }

  template<typename PointT>
    pcl::PointCloud<pcl::Normal>::Ptr
    itnegralImageNormal (const pcl::PointCloud<PointT>& cloud)
    {
      // estimate normals
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

      pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
      ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
      ne.setMaxDepthChangeFactor (0.05f);
      ne.setNormalSmoothingSize (10.0f);
      ne.setInputCloud (cloud.makeShared());
      ne.compute (*normals);
      return normals;
    }

  template<typename PointT>
    pcl::PointCloud<pcl::Normal>::Ptr
    estimateNormals (typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<PointT, pcl::Normal> ne;
      ne.setInputCloud (cloud);

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      typename pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT> ());
      ne.setSearchMethod (tree);

      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.03);

      // Compute the features
      ne.compute (*cloud_normals);
      return cloud_normals;
    }

  template<typename PointT>
    pcl::PointCloud<pcl::Normal>::Ptr
    estimateNormalsOMP (typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
      ne.setInputCloud (cloud);

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      typename pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT> ());
      ne.setSearchMethod (tree);

      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.03);

      // Compute the features
      ne.compute (*cloud_normals);
      return cloud_normals;
    }

  /**
   * converts a point cloud of type PointMo to type PointXYZRGB
   */
  template<typename PointIn, typename PointOut>
    typename pcl::PointCloud<PointOut>::Ptr
    pointmo2PointRGB (pcl::PointCloud<PointIn> input)
    {
      typename pcl::PointCloud<PointOut>::Ptr out (new pcl::PointCloud<PointOut>);
      for (size_t i = 0; i < input.size (); i++)
      {
        PointOut pOut;
        PointIn& pIn = input.points[i];
        pOut.x = pIn.x;
        pOut.y = pIn.y;
        pOut.z = pIn.z;
        pOut.rgb = pIn.rgb;
        out->push_back (pOut);
      }
      return out;
    }

  template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr
    findAndSubtractLine (typename pcl::PointCloud<PointT>::Ptr input, float distance_threshold, float max_iterations)
    {
      typename pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);
      //typedef PointIn XYZ;
      // Find the dominant plane
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients (false);
      seg.setModelType (pcl::SACMODEL_LINE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (distance_threshold);
      seg.setMaxIterations (max_iterations);
      seg.setInputCloud (input);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size () == 0 || coefficients->values.size () == 0)
      {
        return output;
      }
      Eigen::VectorXf model_coefficients;
      model_coefficients.resize (6);
      for (int i = 0; i < 6; i++)
      {
        model_coefficients[i] = coefficients->values[i];
      }

      typename pcl::SampleConsensusModel<PointT>::Ptr model = seg.getModel ();
      model->projectPoints (inliers->indices, model_coefficients, *output, false);
      return output;
    }


  template<typename PointT>
    std::vector<PointT, Eigen::aligned_allocator<PointT> > const
    calcConvexHull (const typename pcl::PointCloud<PointT>::ConstPtr input)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud_hull (new pcl::PointCloud<PointT>);
      pcl::ConvexHull<PointT> chull;
      chull.setInputCloud (input);
      chull.reconstruct (*cloud_hull);
      return cloud_hull->points;
    }
}
#endif /* PCL_COMMONS_H_ */
