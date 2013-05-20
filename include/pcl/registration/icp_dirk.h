/*
 * icp.h
 *
 *  Created on: Jan 30, 2013
 *      Author: elmasry
 */

#ifndef ICP_H_
#define ICP_H_

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>

#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>

#include <pcl/visualization/registration_visualizer.h>

#include "pcl/registration/myicp.h"

namespace pcl
{

  template<typename PointT>
  pcl::CorrespondencesPtr findCorrespondances (typename PointCloud<PointT>::Ptr cloud_source,
      typename PointCloud<PointT>::Ptr cloud_target, float max_dist = 0.05)
  {

    pcl::registration::CorrespondenceEstimation<PointT, PointT> corr_est_;
    corr_est_.setInputTarget (cloud_target);

    pcl::CorrespondencesPtr correspondences_ptr (new pcl::Correspondences);
    corr_est_.setInputSource (cloud_source);
    corr_est_.determineCorrespondences (*correspondences_ptr, max_dist);
    return correspondences_ptr;
  }

  std::vector<int> findRejections (CorrespondencesPtr correspondances_ptr)
  {
    pcl::registration::CorrespondenceRejectorDistance corr_rejection_distance;
    pcl::CorrespondencesPtr correspondences_temp_temp (new pcl::Correspondences);
    std::vector<int> rejected_indices;
    corr_rejection_distance.setInputCorrespondences (correspondances_ptr);
    corr_rejection_distance.setMaximumDistance (0.01f * 0.01f);
    corr_rejection_distance.getCorrespondences (*correspondences_temp_temp);
    corr_rejection_distance.getRejectedQueryIndices (*correspondences_temp_temp, rejected_indices);
    return rejected_indices;
  }

  pcl::PointIndices::Ptr findCorrespondancesIndicesInSource (CorrespondencesPtr correspondances_ptr)
  {
    PointIndices::Ptr indices (new PointIndices);
    for (size_t i = 0; i < correspondances_ptr->size (); i++)
    {
      indices->indices.push_back (correspondances_ptr->at (i).index_query);
    }
    return indices;
  }

  template<typename PointT>
  void icp (typename pcl::PointCloud<PointT>::Ptr cloud_target_ptr,
      typename pcl::PointCloud<PointT>::Ptr cloud_source_ptr, typename pcl::PointCloud<PointT>::Ptr aligned_source,
      Eigen::Matrix4f& final_transform, pcl::CorrespondencesPtr correspondences_ptr, size_t max_iterations = 15)
  {

    typename pcl::PointCloud<PointT>::Ptr cloud_target_ptr_cpy_ (new pcl::PointCloud<PointT>), cloud_source_ptr_cpy_ (
        new pcl::PointCloud<PointT>);

//      pcl::copyPointCloud(*cloud_source_ptr, *cloud_source_ptr_cpy_);
//      copyPointCloud(*cloud_target_ptr, *cloud_target_ptr_cpy_);
    pcl::PassThrough<PointT> pass;
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 1.5);
    pass.setInputCloud (cloud_target_ptr);
    pass.filter (*cloud_target_ptr_cpy_);
    pass.setInputCloud (cloud_source_ptr);
    pass.filter (*cloud_source_ptr_cpy_);

    bool downsampling = true;
    if (downsampling)
    {
      // Create the filtering object
      pcl::VoxelGrid<PointT> sor;
      sor.setLeafSize (0.01f, 0.01f, 0.01f);

      typename pcl::PointCloud<PointT>::Ptr cloud_temp (new pcl::PointCloud<PointT>);

      sor.setInputCloud (cloud_target_ptr_cpy_);
      sor.filter (*cloud_temp);
      cloud_target_ptr_cpy_.swap (cloud_temp);

      sor.setInputCloud (cloud_source_ptr_cpy_);
      sor.filter (*cloud_temp);
      cloud_source_ptr_cpy_.swap (cloud_temp);
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud (*cloud_target_ptr_cpy_, *cloud_target_ptr_cpy_, indices);
    pcl::removeNaNFromPointCloud (*cloud_source_ptr_cpy_, *cloud_source_ptr_cpy_, indices);

//    pcl::io::savePCDFileASCII ("cloud_target.pcd", *(cloud_target_ptr_cpy_));
//    pcl::io::savePCDFileASCII ("cloud_source.pcd", *(cloud_source_ptr_cpy_));

    final_transform = Eigen::Matrix4f::Identity ();

    size_t n_iter = 0;
    float max_dist = 0.01;

    pcl::registration::CorrespondenceEstimation<PointT, PointT> corr_est_;
    corr_est_.setInputTarget (cloud_target_ptr_cpy_);

    printf ("after applying filters: source.size=%d, target.size=%d\n", (int) cloud_source_ptr_cpy_->size (),
        (int) cloud_target_ptr_cpy_->size ());
    printf ("number of iterations=%d\n", max_iterations);
    for (size_t i = 0; i < max_iterations; ++i)
    {
//      correspondences_ptr->clear ();
      printf ("numIteration=%d\n", (int) i);
      corr_est_.setInputSource (cloud_source_ptr_cpy_);
      corr_est_.determineCorrespondences (*correspondences_ptr, max_dist);

      Eigen::Matrix4f transform;
      pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est_;
      trans_est_.estimateRigidTransformation (*cloud_source_ptr_cpy_, *cloud_target_ptr_cpy_, *correspondences_ptr,
          transform);

      printf ("iteration number=%d, correspondances=%d\n", (int) i, (int) correspondences_ptr->size ());
      std::cout << "transform: " << transform << std::endl;
      pcl::transformPointCloud (*cloud_source_ptr_cpy_, *cloud_source_ptr_cpy_, transform);
//      std::cout << "localTransform=" << transform << std::endl;
      final_transform = transform * final_transform;
    }
    std::cout << "final_transform=" << final_transform << std::endl;
    printf ("icp: numCorrespondances=%d\n", (int) correspondences_ptr->size ());
    pcl::copyPointCloud (*cloud_source_ptr, *aligned_source);

  }

  template<typename PointT>
  void icp2 (typename pcl::PointCloud<PointT>::Ptr cloud_target_ptr,
      typename pcl::PointCloud<PointT>::Ptr cloud_source_ptr, Eigen::Matrix4f& final_transform,
      std::vector<int>& rejected_indices)
  {

    typename pcl::PointCloud<PointT>::Ptr cloud_target_ptr_cpy_ (new pcl::PointCloud<PointT>), cloud_source_ptr_cpy_ (
        new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 1.5);
    pass.setInputCloud (cloud_target_ptr);
    pass.filter (*cloud_target_ptr_cpy_);
    pass.setInputCloud (cloud_source_ptr);
    pass.filter (*cloud_source_ptr_cpy_);

    bool downsampling = true;
    if (downsampling)
    {
      // Create the filtering object
      pcl::VoxelGrid<PointT> sor;
      sor.setLeafSize (0.01f, 0.01f, 0.01f);

      typename pcl::PointCloud<PointT>::Ptr cloud_temp (new pcl::PointCloud<PointT>);

      sor.setInputCloud (cloud_target_ptr_cpy_);
      sor.filter (*cloud_temp);
      cloud_target_ptr_cpy_.swap (cloud_temp);

      sor.setInputCloud (cloud_source_ptr_cpy_);
      sor.filter (*cloud_temp);
      cloud_source_ptr_cpy_.swap (cloud_temp);
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud (*cloud_target_ptr_cpy_, *cloud_target_ptr_cpy_, indices);
    pcl::removeNaNFromPointCloud (*cloud_source_ptr_cpy_, *cloud_source_ptr_cpy_, indices);

    final_transform = Eigen::Matrix4f::Identity ();

    pcl::registration::CorrespondenceEstimation<PointT, PointT> correspondence_estimation;
    pcl::registration::CorrespondenceRejectorDistance corr_rejection_distance;
    pcl::CorrespondencesPtr correspondences_temp (new pcl::Correspondences);
    pcl::CorrespondencesPtr correspondences_temp_temp (new pcl::Correspondences);
    correspondence_estimation.setInputCloud (cloud_source_ptr);
    correspondence_estimation.setInputTarget (cloud_target_ptr_cpy_);
    correspondence_estimation.determineCorrespondences (*correspondences_temp, 100.f);
    corr_rejection_distance.setInputCorrespondences (correspondences_temp);
    corr_rejection_distance.setMaximumDistance (0.01f * 0.01f);
#if (!defined PCL_MINOR_VERSION || (PCL_MINOR_VERSION < 3))
    corr_rejection_distance.getCorrespondeces(*correspondences_temp_temp);
#else
    corr_rejection_distance.getCorrespondences (*correspondences_temp_temp);
#endif
    corr_rejection_distance.getRejectedQueryIndices (*correspondences_temp_temp, rejected_indices);
  }

  template<typename PointT>
  void icp4 (typename pcl::PointCloud<PointT>::Ptr cloud_target_ptr_in,
      typename pcl::PointCloud<PointT>::Ptr cloud_source_ptr_in, typename pcl::PointCloud<PointT>::Ptr cloud_aligned,
      Eigen::Matrix4f& final_transform, pcl::CorrespondencesPtr final_correspondances, float max_dist = 1.0f)
  {

    typename pcl::PointCloud<PointT>::Ptr cloud_target_ptr (new pcl::PointCloud<PointT>), cloud_source_ptr (
        new pcl::PointCloud<PointT>);

    copyPointCloud (*cloud_source_ptr_in, *cloud_source_ptr);
    copyPointCloud (*cloud_target_ptr_in, *cloud_target_ptr);

//    pcl::PassThrough<PointT> pass;
//    pass.setFilterFieldName ("x");
//    pass.setFilterLimits (0.0, 1.5);
//    pass.setInputCloud (cloud_target_ptr_in);
//    pass.filter (*cloud_target_ptr);
//    pass.setInputCloud (cloud_source_ptr_in);
//    pass.filter (*cloud_source_ptr);
//
//    bool downsampling = true;
//    if (downsampling)
//    {
//      // Create the filtering object
//      pcl::VoxelGrid<PointT> sor;
//      sor.setLeafSize (0.01f, 0.01f, 0.01f);
//
//      typename pcl::PointCloud<PointT>::Ptr cloud_temp (new pcl::PointCloud<PointT>);
//
//      sor.setInputCloud (cloud_target_ptr);
//      sor.filter (*cloud_temp);
//      cloud_target_ptr.swap (cloud_temp);
//
//      sor.setInputCloud (cloud_source_ptr);
//      sor.filter (*cloud_temp);
//      cloud_source_ptr.swap (cloud_temp);
//    }

//    std::cout << "From target: " << cloud_target_ptr->points.size () << " valid points." << std::endl;
//    std::cout << "From source: " << cloud_source_ptr->points.size () << " valid points." << std::endl;

    final_transform = Eigen::Matrix4f::Identity ();

    unsigned int max_iterations = 15, n_iter = 0;
    bool registration_successful = true;

    pcl::registration::CorrespondenceEstimation<PointT, PointT> corr_est_;
    corr_est_.setInputTarget (cloud_target_ptr);

//    printf("inside icp4: source.size=%d, target.size=%d\n", cloud_source_ptr->size(), cloud_target_ptr->size());
//    pcl::io::savePCDFileASCII (std::string ("cloud_source.pcd"), * cloud_source_ptr);
//    pcl::io::savePCDFileASCII (std::string ("cloud_target.pcd"), * cloud_target_ptr);

    std::vector<float> dist_thresholds;
//    for (int i=0;i<10;i++) dist_thresholds.push_back(1.0f);
//    for (int i=0;i<10;i++) dist_thresholds.push_back(0.5f);
    while (max_dist > 0.01)
    {
      for (int i = 0; i < 10; i++)
        dist_thresholds.push_back (max_dist);
      max_dist /= 1.3;
    }

    for (int i = 0; i < dist_thresholds.size (); i++)
//    while ( (n_iter++ < max_iterations) && registration_successful)
    {
//      std::cout << "ICP iteration " << n_iter << std::endl;

      pcl::CorrespondencesPtr correspondences_ptr (new pcl::Correspondences);
      corr_est_.setInputSource (cloud_source_ptr);
//      corr_est_.determineCorrespondences (*correspondences_ptr, max_dist);
      corr_est_.determineCorrespondences (*correspondences_ptr, dist_thresholds[i]);
//      std::cout << "Found " << correspondences_ptr->size () << " correspondences." << std::endl;

      if (correspondences_ptr->size () == 0)
        break;
//      char filename[255];
//      sprintf (filename, "correspondences_%d.dat", n_iter);
//      std::ofstream file_correspondences (filename, std::ios::trunc);
//      for (size_t i = 0; i < correspondences_ptr->size (); ++i)
//        file_correspondences << correspondences_ptr->at (i) << std::endl;
//      file_correspondences.close ();
//      file_correspondences.clear ();

      Eigen::Matrix4f transform;
      pcl::registration::TransformationEstimationSVD<PointT, PointT> trans_est_;
      trans_est_.estimateRigidTransformation (*cloud_source_ptr, *cloud_target_ptr, *correspondences_ptr, transform);

      pcl::transformPointCloud (*cloud_source_ptr, *cloud_source_ptr, transform);

//      printf ("point.1=(%f,%f,%f)\n", cloud_source_ptr->at (0).x, cloud_source_ptr->at (0).y,
//          cloud_source_ptr->at (0).z);

//      std::cout << "icp transform=" << transform << std::endl;

      final_transform = transform * final_transform;
      final_correspondances->clear ();
      final_correspondances->insert (final_correspondances->end (), correspondences_ptr->begin (),
          correspondences_ptr->end ());
    }
    copyPointCloud (*cloud_source_ptr, *cloud_aligned);
//    std::cout << "icp final transform=" << final_transform << std::endl;
  }

  template<typename PointT>
  void icp5 (typename pcl::PointCloud<PointT>::Ptr cloud_target_ptr_in,
      typename pcl::PointCloud<PointT>::Ptr cloud_source_ptr_in, pcl::CorrespondencesPtr correspondences_ptr)
  {

    float max_dist = 1.0f;
    typename pcl::PointCloud<PointT>::Ptr cloud_target_ptr (new pcl::PointCloud<PointT>), cloud_source_ptr (
        new pcl::PointCloud<PointT>);

    copyPointCloud (*cloud_source_ptr_in, *cloud_source_ptr);
    copyPointCloud (*cloud_target_ptr_in, *cloud_target_ptr);

    pcl::RegistrationVisualizer<PointT, PointT> vis;

    std::vector<int> indicesSrc, indicesTarget;
    for (size_t i = 0; i < correspondences_ptr->size (); i++)
    {
      indicesSrc.push_back ( (*correspondences_ptr)[i].index_query);
      indicesTarget.push_back ( (*correspondences_ptr)[i].index_match);
    }
    vis.updateIntermediateCloud (*cloud_source_ptr, indicesSrc, *cloud_target_ptr, indicesTarget);
    vis.startDisplay ();
  }

}

#endif /* ICP_H_ */
