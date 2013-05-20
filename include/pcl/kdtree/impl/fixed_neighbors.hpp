/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Dirk Holz, University of Bonn.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_KDTREE_KDTREE_IMPL_FIXED_NEIGHBORS_H_
#define PCL_KDTREE_KDTREE_IMPL_FIXED_NEIGHBORS_H_

#include "pcl/kdtree/fixed_neighbors.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>void
pcl::FixedNeighbors<PointT>::setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices)
{
  if (!cloud) return;

  input_   = cloud;
  indices_ = indices;

  int nr_points = (int)(input_->points.size());
  used_.resize(nr_points);

  if (indices_ == NULL)
  {
    for (int i = 0; i < nr_points; ++i)
      used_[i] = true;
  }
  else
  {
    for (int i = 0; i < nr_points; ++i)
      used_[i] = false;
    const int nr_indices = (int)(indices_->size());
    const std::vector<int>& indices = *indices_;
    for (int i = 0; i < nr_indices; ++i)
      used_[indices[i]] = true;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::FixedNeighbors<PointT>::radiusSearch (
    const PointCloud& cloud_arg,
    int index_arg,
    double radius_arg,
    std::vector<int> &k_indices_arg,
    std::vector<float> &k_sqr_distances_arg,
    int max_nn_arg) const
{
  if (!neighbors_)
  {
    PCL_ERROR ("[pcl::%s::radiusSearch] Input neighbor set does not exist!\n", getName ().c_str ());
    return (false);
  }

  if ( (index_arg < 0) || (index_arg > (int)neighbors_->size()) )
  {
    PCL_ERROR ("[pcl::%s::radiusSearch] Index out of range!\n", getName ().c_str ());
    return (false);
  }

  k_indices_arg.clear();
  k_sqr_distances_arg.clear();

  if (!used_[index_arg])
    return 0;

  // include query point
  k_indices_arg.push_back(index_arg);
  k_sqr_distances_arg.push_back(0.0f);

  const PointT& query_point = cloud_arg.points[index_arg];
  const std::vector<int>& neighbor_vector = neighbors_->at(index_arg);
  int nr_neighbors = (int)neighbor_vector.size();

  float squared_radius = (float)(radius_arg*radius_arg);
  for (int i = 0; i < nr_neighbors; ++i)
  {
    const int idx_neighbor = neighbor_vector[i];
    if (!used_[idx_neighbor])
      continue;
    const PointT& neighbor = cloud_arg.points[idx_neighbor];
    const float dx = neighbor.x - query_point.x;
    const float dy = neighbor.y - query_point.y;
    const float dz = neighbor.z - query_point.z;
    const float squared_distance = (dx*dx + dy*dy + dz*dz);
    if (squared_distance <= squared_radius)
    {
      k_indices_arg.push_back(idx_neighbor);
      k_sqr_distances_arg.push_back(squared_distance);
    }

    if ((int)(k_indices_arg.size() +1) >= max_nn_arg)
      break;
  }

  return (int)(k_indices_arg.size());
}




//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::FixedNeighbors<PointT>::nearestKSearch (
    const PointCloud& cloud_arg,
    int index_arg,
    int k_arg,
    std::vector<int> &k_indices_arg,
    std::vector<float> &k_sqr_distances_arg) const
{
  if (!neighbors_)
  {
    PCL_ERROR ("[pcl::%s::radiusSearch] Input neighbor set does not exist!\n", getName ().c_str ());
    return (false);
  }

  if ( (index_arg < 0) || (index_arg > (int)neighbors_->size()) )
  {
    PCL_ERROR ("[pcl::%s::radiusSearch] Index out of range!\n", getName ().c_str ());
    return (false);
  }

  k_indices_arg.clear();
  k_sqr_distances_arg.clear();

  if (!used_[index_arg])
    return 0;

  // include query point
  k_indices_arg.push_back(index_arg);
  k_sqr_distances_arg.push_back(0.0f);

  const PointT& query_point = cloud_arg.points[index_arg];
  const std::vector<int>& neighbor_vector = neighbors_->at(index_arg);
  int nr_neighbors = (int)neighbor_vector.size();

  int max_k = std::min(k_arg, nr_neighbors);
  if (max_k < 0)
    return 1;

  for (int i = 0; i < max_k; ++i)
  {
    const int idx_neighbor = neighbor_vector[i];
    if (!used_[idx_neighbor])
      continue;
    const PointT& neighbor = cloud_arg.points[idx_neighbor];
    const float dx = neighbor.x - query_point.x;
    const float dy = neighbor.y - query_point.y;
    const float dz = neighbor.z - query_point.z;
    const float squared_distance = (dx*dx + dy*dy + dz*dz);
    k_indices_arg.push_back(idx_neighbor);
    k_sqr_distances_arg.push_back(squared_distance);
  }

  return (int)(k_indices_arg.size());
}

#define PCL_INSTANTIATE_FixedNeighbors(T) template class PCL_EXPORTS pcl::FixedNeighbors<T>;

#endif //#ifndef PCL_KDTREE_KDTREE_IMPL_FIXED_NEIGHBORS_H_
