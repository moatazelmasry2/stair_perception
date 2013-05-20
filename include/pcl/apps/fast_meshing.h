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
 * $Id:$
 *
 */

#ifndef PCL_APPS_FAST_MESHING_H_
#define PCL_APPS_FAST_MESHING_H_

//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include "pcl/filters/impl/filter.hpp"

#include <pcl/surface/gp3.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/surface_functions.h>

#include "pcl/surface/simplification_remove_unused_vertices.h"

namespace pcl
{

  template <typename PointInT, typename PointOutT>
  class FastMeshing
  {
    public:
      typedef pcl::PointCloud<PointInT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      typedef typename pcl::KdTree<PointInT> SearchTree;

      FastMeshing(){};
      ~FastMeshing(){};

      void mesh(const PointCloudConstPtr &cloud, pcl::PolygonMesh::Ptr& mesh, std::vector<int>& indices)
      {
        //pcl::StopWatch timer;

        bool use_organized_fast_mesh = true;
        if (cloud->height == 1)
          use_organized_fast_mesh = false;

        std::vector<int> indices_correct;

        if (use_organized_fast_mesh)
        {
          //printf("Reconstructing surface using OrganizedFastMesh...\n");
//          timer.reset();

          int polygon_edge_length = 1;
          if (cloud->width == 640)
            polygon_edge_length = 4;
          else if  (cloud->width == 320)
            polygon_edge_length = 3;

          // hack
          //polygon_edge_length = 10;
          pcl::OrganizedFastMesh<PointInT> fast_mesh;
          fast_mesh.setInputCloud(cloud);
          fast_mesh.setTrianglePixelSize(polygon_edge_length);
//          fast_mesh.setTriangulationType(fast_mesh.QUAD_MESH);
          fast_mesh.setTriangulationType(fast_mesh.TRIANGLE_ADAPTIVE_CUT);
          fast_mesh.reconstruct(*mesh);
         // printf("Computing organized fast mesh took %0.2fms (%d vertices, %d triangles).\n",
         //        timer.getTime(),
         //        mesh->cloud.height*mesh->cloud.width,
         //        (int)mesh->polygons.size());

        }
        else // Given point cloud is not organized, using alternative solution for unorganized clouds
        {
          // filter cloud for gp3
//          timer.reset();
          PointCloudPtr cloud_input_filtered (new PointCloud());

          pcl::removeNaNFromPointCloud(*cloud, *cloud_input_filtered, indices_correct);
//          printf("Removing NaNs took %0.2lfms\n", timer.getTime());


          // Normal estimation*
//          timer.reset();
          int nr_neighbors = 20;
          pcl::NormalEstimation<PointInT, pcl::Normal> n;
          pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
          //typename pcl::KdTree<PointInT>::Ptr tree (new pcl::KdTreeFLANN<PointInT>);
          typename pcl::search::KdTree<PointInT>::Ptr tree (new pcl::search::KdTree<PointInT>);
          tree->setInputCloud (cloud_input_filtered);
          n.setInputCloud (cloud_input_filtered);
          n.setSearchMethod (tree);
          n.setKSearch (nr_neighbors);
          n.compute (*normals);

          typename pcl::PointCloud<PointOutT>::Ptr cloud_with_normals (new pcl::PointCloud<PointOutT>);
          pcl::concatenateFields (*cloud_input_filtered, *normals, *cloud_with_normals);
//          printf("Computing normals (using %d neighbors) took %0.2fms\n", nr_neighbors, timer.getTime());


          printf("Reconstructing surface using Greedy Projection...\n");
//          timer.reset();
          //typename pcl::KdTree<PointOutT>::Ptr tree2 (new pcl::KdTreeFLANN<PointOutT>);
          typename pcl::search::KdTree<PointOutT>::Ptr tree2 (new pcl::search::KdTree<PointOutT>);
          tree2->setInputCloud (cloud_with_normals);

          pcl::GreedyProjectionTriangulation<PointOutT> gp3;
//          gp3.setSearchRadius (0.025);
          gp3.setSearchRadius (0.1);
//          gp3.setSearchRadius (0.5);
          gp3.setMu (2.5);
          gp3.setMaximumNearestNeighbors (100);
//          gp3.setMaximumSurfaceAgle(M_PI/4); // 45 degrees
          gp3.setMinimumAngle(M_PI/18); // 10 degrees
          gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
          gp3.setNormalConsistency(false);
          gp3.setInputCloud (cloud_with_normals);
          gp3.setSearchMethod (tree2);
          gp3.reconstruct (*mesh);

//          printf("Computing mesh using GP3 took %0.2fms.\n", timer.getTime());
        }


        /// remove unused points
//        timer.reset();
        pcl::PolygonMesh::Ptr mesh_filtered(new pcl::PolygonMesh());
        pcl::surface::SimplificationRemoveUnusedVertices filter_vertices;
        filter_vertices.simplify(*mesh, *mesh_filtered, indices);
        mesh.swap(mesh_filtered);
        if ( !use_organized_fast_mesh )
          for ( unsigned int i = 0; i < indices.size(); ++i)
            indices[i] = indices_correct[indices[i]];
        //printf("Removing unused vertices took %0.2fms (%d vertices, %d triangles).\n",
        //       timer.getTime(),
        //       mesh->cloud.height*mesh->cloud.width,
        //       (int)mesh->polygons.size());

      } /* void mesh(const PointCloudConstPtr &cloud, pcl::PolygonMesh::Ptr& mesh) */
  };
}

#define PCL_INSTANTIATE_FastMeshing(In,Out) template class PCL_EXPORTS pcl::FastMeshing<In,Out>;

#endif /* PCL_APPS_FAST_MESHING_H_ */
