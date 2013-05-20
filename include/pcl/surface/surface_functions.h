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

#ifndef PCL_SURFACE_FUNCTIONS_H_
#define PCL_SURFACE_FUNCTIONS_H_

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>

#include <pcl/common/mesh_utilities.h>
#include <pcl/common/incremental_plane_fitting.h>

#include <pcl/apps/fast_meshing.h>


// #include <pcl/segmentation/normal_segmentation.h>

namespace pcl
{
  namespace surface
  {

//    template <typename PointType> inline void
//    correctNormal(const PointType& point, const Eigen::Vector3f& viewpoint = Eigen::Vector3f::Zero())
//    {
//      correctNormal(point, point, viewpoint);
//    }
//
//    template <typename PointType, typename NormalType> inline void
//    correctNormal(const PointType& point, NormalType& normal, const Eigen::Vector3f& viewpoint = Eigen::Vector3f::Zero())
//    {
//      normal.getNormalVector3fMap().normalize();
//      if ((viewpoint - point.getVector3fMap()).dot(normal.getNormalVector3fMap()) < 0)
//        normal.getNormalVector3fMap() *= -1;
//    }
//
//    template <typename PointType> inline void
//    correctNormalAndComputeCurvature(const PointType& point, const Eigen::Vector3f& viewpoint = Eigen::Vector3f::Zero())
//    {
//      correctNormalAndComputeCurvature(point, point, viewpoint);
//    }
//
//    template <typename PointType, typename NormalType> inline void
//    correctNormalAndComputeCurvature(const PointType& point, NormalType& normal, const Eigen::Vector3f& viewpoint = Eigen::Vector3f::Zero())
//    {
//      normal.getNormalVector3fMap().normalize();
//
//      if ((viewpoint - point.getVector3fMap()).dot(normal.getNormalVector3fMap()) < 0)
//        normal.getNormalVector3fMap() *= -1;
//
//    }

    template<typename PointT, typename PointNT> inline void computeNormalsAreaWeighted (
        const pcl::PointCloud<PointT>& cloud, const std::vector<pcl::Vertices>& polygons,
        pcl::PointCloud<PointNT>& normals)
    {
      int nr_points = (int) cloud.points.size ();
      int nr_polygons = (int) polygons.size ();

      normals.header = cloud.header;
      normals.width = cloud.width;
      normals.height = cloud.height;
      normals.points.resize (nr_points);

      for (int i = 0; i < nr_points; ++i)
        normals.points[i].getNormalVector3fMap () = Eigen::Vector3f::Zero ();

      // NOTE: for efficiency the weight is computed implicitly by using the
      // cross product, this causes inaccurate normals for meshes containing
      // non-triangle polygons (quads or other types)
      for (int i = 0; i < nr_polygons; ++i)
      {
        const int nr_points_polygon = (int) polygons[i].vertices.size ();
        if (nr_points_polygon < 3)
          continue;

        // compute normal for triangle
        Eigen::Vector3f vec_a_b = cloud.points[polygons[i].vertices[0]].getVector3fMap ()
            - cloud.points[polygons[i].vertices[1]].getVector3fMap ();
        Eigen::Vector3f vec_a_c = cloud.points[polygons[i].vertices[0]].getVector3fMap ()
            - cloud.points[polygons[i].vertices[2]].getVector3fMap ();
        Eigen::Vector3f normal = vec_a_b.cross (vec_a_c);
        pcl::flipNormalTowardsViewpoint (cloud.points[polygons[i].vertices[0]], 0.0f, 0.0f, 0.0f, normal (0),
            normal (1), normal (2));

        // add normal to all points in polygon
        for (int j = 0; j < nr_points_polygon; ++j)
          normals.points[polygons[i].vertices[j]].getNormalVector3fMap () += normal;
      }

      for (int i = 0; i < nr_points; ++i)
      {
        normals.points[i].getNormalVector3fMap ().normalize ();
        pcl::flipNormalTowardsViewpoint (cloud.points[i], 0.0f, 0.0f, 0.0f, normals.points[i].normal_x,
            normals.points[i].normal_y, normals.points[i].normal_z);
      }
    }

    template<typename PointT, typename PointNT> inline void computeNormalsAreaWeightedInverse (
        const pcl::PointCloud<PointT>& cloud, const std::vector<pcl::Vertices>& polygons,
        pcl::PointCloud<PointNT>& normals)
    {
      int nr_points = (int) cloud.points.size ();
      int nr_polygons = (int) polygons.size ();

      normals.header = cloud.header;
      normals.width = cloud.width;
      normals.height = cloud.height;
      normals.points.resize (nr_points);

      for (int i = 0; i < nr_points; ++i)
        normals.points[i].getNormalVector3fMap () = Eigen::Vector3f::Zero ();

      for (int i = 0; i < nr_polygons; ++i)
      {
        const int nr_points_polygon = (int) polygons[i].vertices.size ();
        if (nr_points_polygon < 3)
          continue;

        // compute normal for triangle
        Eigen::Vector3f vec_a_b = cloud.points[polygons[i].vertices[0]].getVector3fMap ()
            - cloud.points[polygons[i].vertices[1]].getVector3fMap ();
        Eigen::Vector3f vec_a_c = cloud.points[polygons[i].vertices[0]].getVector3fMap ()
            - cloud.points[polygons[i].vertices[2]].getVector3fMap ();
        Eigen::Vector3f normal = vec_a_b.cross (vec_a_c);
        pcl::flipNormalTowardsViewpoint (cloud.points[polygons[i].vertices[0]], 0.0f, 0.0f, 0.0f, normal (0),
            normal (1), normal (2));

        float squared_norm = normal.squaredNorm ();
        normal /= squared_norm;

        // add normal to all points in polygon
        for (int j = 0; j < nr_points_polygon; ++j)
          normals.points[polygons[i].vertices[j]].getNormalVector3fMap () += normal;
      }

      for (int i = 0; i < nr_points; ++i)
      {
        normals.points[i].getNormalVector3fMap ().normalize ();
        pcl::flipNormalTowardsViewpoint (cloud.points[i], 0.0f, 0.0f, 0.0f, normals.points[i].normal_x,
            normals.points[i].normal_y, normals.points[i].normal_z);
      }
    }

    template<typename PointT> inline void computeApproximateCurvature (pcl::PointCloud<PointT>& cloud,
        const pcl::NeighborhoodVector& neighbors)
    {
      const int nr_points = (int) cloud.points.size ();
      for (int idx_point = 0; idx_point < nr_points; ++idx_point)
      {
        const int nr_neighbors = (int) neighbors[idx_point].size ();
        if (nr_neighbors == 0)
          continue;

        PointT& point = cloud.points[idx_point];

        Eigen::Vector3f centroid = Eigen::Vector3f::Zero ();
        for (int idx_neighbor = 0; idx_neighbor < nr_neighbors; ++idx_neighbor)
          centroid += cloud.points[neighbors[idx_point][idx_neighbor]].getVector3fMap ();
        centroid /= (float) nr_neighbors;

//        Eigen::Vector4f centroid;
//        pcl::compute3DCentroid(cloud, neighbors[idx_point], centroid);

//        pcl::PlaneModel plane_model;
//        plane_model.fromNormalAndCentroid(point.getNormalVector3fMap(), centroid);
//        float residual_distance = pcl::distPointPlane(point, plane_model);

        const float distance = -point.getNormalVector3fMap ().dot (centroid.head<3> ());
//        const float distance = -point.getNormalVector3fMap().dot(centroid);
        const float residual_distance = point.getNormalVector3fMap ().dot (point.getVector3fMap ()) + distance;
        point.curvature = residual_distance;
      }
    }

    template<typename PointT> inline void getCloud (const pcl::PolygonMesh& mesh, pcl::PointCloud<PointT>& cloud)
    {
      pcl::fromROSMsg (mesh.cloud, cloud);
    }

    inline float fast_exp (double number)
    {
      float result;
      * ((int*) (&result) + 0) = 0;
      * ((int*) (&result) + 1) = (int) (1512775 * number + 1072632447);
      return result;
    }

    float fast_sqrt (float number)
    {
      long i;
      float x2, y;
      const float threehalfs = 1.5F;
      x2 = number * 0.5F;
      y = number;
      i = *(long *) &y;  // evil floating point bit level hacking
      i = 0x5f3759df - (i >> 1);  // what the fuck?
      y = *(float *) &i;
      y = y * (threehalfs - (x2 * y * y));  // 1st iteration
      return y;
    }

    /**
     * @brief NEW distance term
     */
    template<typename PointAT, typename PointBT>
    inline float temp_dist_term (const PointAT& point_a, const PointBT& point_b, const float& w1, const float& w2,
        const float& w3)
    {
      const float z0 = point_a.x - point_b.x;
      const float z1 = point_a.y - point_b.y;
      const float z2 = point_a.z - point_b.z;

      const float z3 = point_a.normal_x - point_b.normal_x;
      const float z4 = point_a.normal_y - point_b.normal_y;
      const float z5 = point_a.normal_z - point_b.normal_z;

      const float zA = std::abs (z0 * point_a.normal_x + z1 * point_a.normal_y + z2 * point_a.normal_z);
      const float zB = std::abs (z0 * point_b.normal_x + z1 * point_b.normal_y + z2 * point_b.normal_z);

      const float distance_scaling = std::max (1.0f, (point_a.z + point_b.z) * (point_a.z + point_b.z));

      float dist = exp (
          1.0f / distance_scaling
              * (-w1 * sqrt (z0 * z0 + z1 * z1 + z2 * z2) - w2 * sqrt (z3 * z3 + z4 * z4 + z5 * z5)
                  - w3 * std::max (zA, zB)));

      return dist;
    }

    template<typename PointAT, typename PointBT>
    inline float new_dist_term (const PointAT& point_a, const PointBT& point_b, const float& w1, const float& w2,
        const float& w3)
    {
      const float dist_points = (point_a.getVector3fMap () - point_b.getVector3fMap ()).norm ();

      Eigen::Vector3f diff_normals = point_a.getNormalVector3fMap () - point_b.getNormalVector3fMap ();
      const float dist_normals = diff_normals.lpNorm<1> ();

      const float dist_scaling = 1.0f / std::max (1.0f, (point_a.z + point_b.z) * (point_a.z + point_b.z));

      return (exp (dist_scaling * (-w1 * dist_points - w2 * dist_normals)));
//      return (exp( (-w1*dist_points - w2*dist_normals)));
    }

    template<typename PointInT, typename PointOutT> inline void smoothBilateral (const pcl::PointCloud<PointInT>& input,
        const std::vector<pcl::Vertices>& vertex_list, pcl::PointCloud<PointOutT>& output)
    {
      output.header = input.header;
      output.width = input.width;
      output.height = input.height;
      output.points.resize (input.points.size ());

      float w1 = 10, w2 = 0.1, w3 = 100;

      for (unsigned int i = 0; i < vertex_list.size (); ++i)
      {
        if (vertex_list[i].vertices.size () == 0)
          continue;

        const PointInT& point_in = input.points[i];
        PointOutT& point_out = output.points[i];
//        point_out.x = point_in.x;
//        point_out.y = point_in.y;
//        point_out.z = point_in.z;
//        point_out.normal_x = point_in.normal_x;
//        point_out.normal_y = point_in.normal_y;
//        point_out.normal_z = point_in.normal_z;
//        float weight = 0.0f, sum_weights = 1.0f;
        point_out.x = 0.0f;
        point_out.y = 0.0f;
        point_out.z = 0.0f;
        point_out.normal_x = 0.0f;
        point_out.normal_y = 0.0f;
        point_out.normal_z = 0.0f;
        float weight = 0.0f, sum_weights = 0.0f;

////        // HACK: skip point if it's not a shared vertex
////        if ( vertex_list[i].vertices.size() < 8 )
////          continue;
//          if ( vertex_list[i].vertices.size() < 12 )
//          {
//            point_out.x = point_in.x;
//            point_out.y = point_in.y;
//            point_out.z = point_in.z;
//            point_out.normal_x = point_in.normal_x;
//            point_out.normal_y = point_in.normal_y;
//            point_out.normal_z = point_in.normal_z;
//            continue;
//          }

        for (unsigned int j = 0; j < vertex_list[i].vertices.size (); ++j)
        {
          const PointInT& neighbor = input.points[vertex_list[i].vertices[j]];
//          weight = temp_dist_term(point_in, neighbor, w1, w2, w3);
          weight = new_dist_term (point_in, neighbor, 1.0f, 1.0f, 0.0f);
          sum_weights += weight;

          point_out.x += weight * neighbor.x;
          point_out.y += weight * neighbor.y;
          point_out.z += weight * neighbor.z;
          point_out.normal_x += weight * neighbor.normal_x;
          point_out.normal_y += weight * neighbor.normal_y;
          point_out.normal_z += weight * neighbor.normal_z;
        }
        if (sum_weights > 0.0f)
        {
          point_out.x /= sum_weights;
          point_out.y /= sum_weights;
          point_out.z /= sum_weights;
          point_out.getNormalVector3fMap ().normalize ();
        }

//        point_out.curvature = (float)(vertex_list[i].vertices.size());
      }
    }

    template<typename PointInT, typename PointOutT> inline void smoothBilateralNEW (
        const pcl::PointCloud<PointInT>& input, const std::vector<std::vector<int> >& vertex_list,
        pcl::PointCloud<PointOutT>& output)
    {
      output.header = input.header;
      output.width = input.width;
      output.height = input.height;
      output.points.resize (input.points.size ());

      //float w1 = 10, w2 = 0.1, w3 = 100;

      for (unsigned int i = 0; i < vertex_list.size (); ++i)
      {
        if (vertex_list[i].size () == 0)
          continue;

        const PointInT& point_in = input.points[i];
        PointOutT& point_out = output.points[i];
        point_out.x = 0.0f;
        point_out.y = 0.0f;
        point_out.z = 0.0f;
        point_out.normal_x = 0.0f;
        point_out.normal_y = 0.0f;
        point_out.normal_z = 0.0f;
        float weight = 0.0f, sum_weights = 0.0f;

        for (unsigned int j = 0; j < vertex_list[i].size (); ++j)
        {
          const PointInT& neighbor = input.points[vertex_list[i][j]];
          weight = new_dist_term (point_in, neighbor, 1.0f, 1.0f, 0.0f);
          sum_weights += weight;

          point_out.x += weight * neighbor.x;
          point_out.y += weight * neighbor.y;
          point_out.z += weight * neighbor.z;
          point_out.normal_x += weight * neighbor.normal_x;
          point_out.normal_y += weight * neighbor.normal_y;
          point_out.normal_z += weight * neighbor.normal_z;
        }
        if (sum_weights > 0.0f)
        {
          point_out.x /= sum_weights;
          point_out.y /= sum_weights;
          point_out.z /= sum_weights;
          point_out.getNormalVector3fMap ().normalize ();
        }
      }
    }

    inline void filterMesh (const pcl::PolygonMesh& input, pcl::PolygonMesh& output, const float& x_min,
        const float& x_max, const float& y_min, const float& y_max, const float& z_min, const float& z_max)
    {
      int nr_points = input.cloud.width * input.cloud.height;
      if (nr_points == 0)
        return;

      int idx_x = -1, idx_y = -1, idx_z = -1;
      for (size_t d = 0; d < input.cloud.fields.size (); ++d)
      {
        if (input.cloud.fields[d].name == "x")
          idx_x = d;
        else if (input.cloud.fields[d].name == "y")
          idx_y = d;
        else if (input.cloud.fields[d].name == "z")
          idx_z = d;
      }
      if ( (idx_x == -1) || (idx_y == -1) || (idx_z == -1))
      {
        PCL_ERROR("%s: Missing field name for xyz!\n", __PRETTY_FUNCTION__);
        return;
      }

      output.header = input.header;

      /// filter cloud ///////////////////////////////////////////////////////
      output.cloud.width = input.cloud.width;
      output.cloud.height = 1;
      output.cloud.fields = input.cloud.fields;
      output.cloud.is_dense = false;
      output.cloud.row_step = input.cloud.row_step;
      output.cloud.point_step = input.cloud.point_step;
      output.cloud.is_bigendian = input.cloud.is_bigendian;
      output.cloud.data.resize (input.cloud.data.size ());

      Eigen::Vector4f pt = Eigen::Vector4f::Zero ();
      Eigen::Array4i xyz_offset (input.cloud.fields[idx_x].offset, input.cloud.fields[idx_y].offset,
          input.cloud.fields[idx_z].offset, 0);
      std::vector<int> new_index (nr_points, -1);  // all points get 'invalid' index
      int nr_p = 0;
      for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input.cloud.point_step)
      {
        memcpy (&pt[0], &input.cloud.data[xyz_offset[0]], sizeof(float));
        memcpy (&pt[1], &input.cloud.data[xyz_offset[1]], sizeof(float));
        memcpy (&pt[2], &input.cloud.data[xyz_offset[2]], sizeof(float));

        if (!pcl_isfinite(pt[0])|| !pcl_isfinite(pt[1]) || !pcl_isfinite(pt[2]))
        continue;
        if ( (pt[0] < x_min) || (pt[0] > x_max) || (pt[1] < y_min) || (pt[1] > y_max) || (pt[2] < z_min)
            || (pt[2] > z_max))
          continue;

        // copy all fields from 'input' to 'output'
        new_index[cp] = nr_p;
        memcpy (&output.cloud.data[nr_p * output.cloud.point_step], &input.cloud.data[cp * output.cloud.point_step],
            output.cloud.point_step);
        nr_p++;
      }
      output.cloud.width = nr_p;
      output.cloud.row_step = output.cloud.point_step * output.cloud.width;
      output.cloud.data.resize (output.cloud.width * output.cloud.height * output.cloud.point_step);

      /// update polygons ////////////////////////////////////////////////////
      output.polygons.clear ();
      output.polygons.reserve (input.polygons.size ());
      pcl::Vertices polygon;
      for (unsigned int i = 0; i < input.polygons.size (); ++i)
      {
        polygon.vertices.resize (input.polygons[i].vertices.size ());
        bool polygon_in_box = true;
        for (unsigned int j = 0; j < input.polygons[i].vertices.size (); ++j)
        {
          const int& idx = input.polygons[i].vertices[j];
          if (idx == -1)
          {
            polygon_in_box = false;
            break;
          }
          polygon.vertices[j] = idx;
        }
        if (polygon_in_box)
          output.polygons.push_back (polygon);
      }

    }

    template<typename PointT> inline float getIntensity (PointT& point)
    {
      return ((float) (0.299 * (float) (point.r) / 255.0f + 0.587 * (float) (point.g) / 255.0f
          + 0.114 * (float) point.b / 255.0f));
    }

    template<typename PointT> inline void makeNormalColors (pcl::PointCloud<PointT>& cloud)
    {
      for (unsigned int i = 0; i < cloud.points.size (); ++i)
      {
        PointT& point = cloud.points[i];
        point.r = (uint8_t) (255.0f * 0.5f * (1.0f + point.normal_x));
        point.g = (uint8_t) (255.0f * 0.5f * (1.0f + point.normal_y));
        point.b = (uint8_t) (255.0f * 0.5f * (1.0f + point.normal_z));
      }
    }
  } /* namespace surface */
} /* namespace pcl */

#endif /* PCL_SURFACE_FUNCTIONS_H_ */
