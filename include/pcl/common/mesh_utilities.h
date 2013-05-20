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

#ifndef PCL_COMMON_NEIGHBORS_FROM_MESH_H_
#define PCL_COMMON_NEIGHBORS_FROM_MESH_H_

#include <vector>
#include <queue>
#include <boost/shared_ptr.hpp>
#include <pcl/Vertices.h>
#include <pcl/PolygonMesh.h>

namespace pcl
{

  typedef int NeighborType;
  typedef std::vector<NeighborType> Neighborhood;
  typedef std::vector<Neighborhood> NeighborhoodVector;
  typedef boost::shared_ptr< NeighborhoodVector > NeighborhoodVectorPtr;
  typedef boost::shared_ptr< const NeighborhoodVector > NeighborhoodVectorConstPtr;

  typedef float DistanceType;
  typedef std::vector<DistanceType> NeighborDistances;
  typedef std::vector<NeighborDistances> NeighborDistancesVector;
  typedef boost::shared_ptr< NeighborDistancesVector > NeighborDistancesVectorPtr;
  typedef boost::shared_ptr< const NeighborDistancesVector > NeighborDistancesVectorConstPtr;

  inline void
  addNeighborAllowingDuplicates(pcl::Neighborhood& neighborhood, const int& neighbor)
  {
    neighborhood.push_back(neighbor);
  }

  inline void
  addNeighbor(pcl::Neighborhood& neighborhood, const int& neighbor)
  {
    const int neighborhood_size = (int)(neighborhood.size());
    for (int i = 0; i < neighborhood_size; ++i)
      if(neighborhood[i] == neighbor)
        return;
    neighborhood.push_back(neighbor);
  }



  inline void
  getVertexVertexList(const std::vector<pcl::Vertices>& polygons, const int& nr_points, pcl::NeighborhoodVector& vertex_list, bool ignore_dupilicates = true)
  {
    vertex_list.clear();
    int nr_polygons = polygons.size();
    if ( nr_polygons == 0 )
      return;

    vertex_list.resize(nr_points);
    for ( int i = 0; i < nr_points; ++i)
      vertex_list[i].clear();

    if (ignore_dupilicates)
    {
      for (int idx_poly = 0; idx_poly < nr_polygons; ++idx_poly)
      {
        const int idx_last_point = (int)(polygons[idx_poly].vertices.size() - 1);
        for (int idx_point = 0; idx_point < idx_last_point; ++idx_point)
        {
          const int point = polygons[idx_poly].vertices[idx_point];
          const int next_point = polygons[idx_poly].vertices[idx_point+1];
          addNeighbor(vertex_list[point], next_point);
          addNeighbor(vertex_list[next_point], point);
        }
        const int first_point = polygons[idx_poly].vertices[0];
        const int last_point = polygons[idx_poly].vertices[idx_last_point];
        addNeighbor(vertex_list[last_point], first_point);
        addNeighbor(vertex_list[first_point], last_point);
      }
    }
    else
    {
      for (int idx_poly = 0; idx_poly < nr_polygons; ++idx_poly)
      {
        const int idx_last_point = (int)(polygons[idx_poly].vertices.size() - 1);
        for (int idx_point = 0; idx_point < idx_last_point; ++idx_point)
        {
          const int point = polygons[idx_poly].vertices[idx_point];
          const int next_point = polygons[idx_poly].vertices[idx_point+1];
          vertex_list[point].push_back(next_point);
          vertex_list[next_point].push_back(point);
        }
        const int first_point = polygons[idx_poly].vertices[0];
        const int last_point = polygons[idx_poly].vertices[idx_last_point];
        vertex_list[last_point].push_back(first_point);
        vertex_list[first_point].push_back(last_point);
      }
    }
  }




  inline void
  getVertexVertexList(const pcl::PolygonMesh& mesh, pcl::NeighborhoodVector& vertex_list, bool ignore_duplicates = true)
  {
    const std::vector<pcl::Vertices>& polygons = mesh.polygons;
    const int nr_points = mesh.cloud.width * mesh.cloud.height;
    getVertexVertexList(polygons, nr_points, vertex_list, ignore_duplicates);
  }




  inline void
  computeRingNeighborhood (const pcl::NeighborhoodVector& vertex_vertex_list, pcl::NeighborhoodVector& ring_neighborhood, const int neighborhood_size = 1)
  {
    const int nr_vertices = (int)vertex_vertex_list.size();
    const int size_of_k = std::max(neighborhood_size + 1, 0);

    // initialize results
    ring_neighborhood.resize(nr_vertices);
    pcl::NeighborhoodVector ring_neighborhood_limits;
    ring_neighborhood_limits.resize(nr_vertices);

    // initialize temporary memory
    std::vector<bool> used_in_current_neighborhood(nr_vertices, false);

    for (int idx_vertex = 0; idx_vertex < nr_vertices; ++idx_vertex)
    {
      pcl::Neighborhood& neighborhood = ring_neighborhood[idx_vertex];
      neighborhood.clear(); // should be empty already
      neighborhood.push_back(idx_vertex); // neighbor in ring 0 is the point itself

      pcl::Neighborhood& neighborhood_limits = ring_neighborhood_limits[idx_vertex];
      neighborhood_limits.clear();
      neighborhood_limits.reserve(size_of_k);
      neighborhood_limits.push_back((int)neighborhood.size());

      int idx_current_ring = 0;
      int idx_last_ring_begin = 0;
      int idx_last_ring_end = neighborhood_limits.back();
      while (++idx_current_ring < size_of_k)
      {

        for (int idx_neighbor = idx_last_ring_begin; idx_neighbor < idx_last_ring_end; ++idx_neighbor)
        {
          // retrieve neighbor's neighborhood
          const pcl::Neighborhood& neighbor_neighborhood = vertex_vertex_list[neighborhood[idx_neighbor]];
          const int size_neighbor_neighborhood = (int)neighbor_neighborhood.size();
          if (size_neighbor_neighborhood == 0)
            continue;

          for (int idx_neighbor_neighbor = 0; idx_neighbor_neighbor < size_neighbor_neighborhood; ++idx_neighbor_neighbor)
          {
            const int& neighbor_neighbor = neighbor_neighborhood[idx_neighbor_neighbor];
            if (!used_in_current_neighborhood[neighbor_neighbor])
            {
              neighborhood.push_back(neighbor_neighbor);
              used_in_current_neighborhood[neighbor_neighbor] = true;
            }
          }
        }
        neighborhood_limits.push_back((int)neighborhood.size());

        idx_last_ring_begin = idx_last_ring_end;
        idx_last_ring_end = neighborhood_limits.back();
      }

      int final_neighborhood_size = (int)neighborhood.size();
      for (int i = 0; i < final_neighborhood_size; ++i)
        used_in_current_neighborhood[neighborhood[i]] = false;

//      printf("%d now has %d neighbors (instead of %d)!\n", idx_vertex, (int)neighborhood.size(), (int)vertex_vertex_list[idx_vertex].size());
    }


  }





//  inline void
//  removeUnusedVertices(const std::vector<pcl::Vertices>& in_polygons, std::vector<pcl::Vertices>& out_polygons, int nr_points)
//  {
//    if ( in_polygons.size() == 0)
//      return;
//
//    std::vector<bool> new_indices(nr_points, false);
//
//    std::vector<int> indices;
//    indices.clear();
//    indices.reserve(nr_points);
//
//    // mark all points in triangles as being used
//    const int nr_polygons = (int)in_polygons.size();
//    for ( int polygon = 0; polygon < nr_polygons; ++polygon)
//    {
//      const int nr_points_polygon = (int)in_polygons[polygon].vertices.size();
//      for ( int point = 0; point < nr_points_polygon; ++point )
//        if ( new_indices[ in_polygons[polygon].vertices[point] ] == -1 )
//        {
//          new_indices[ in_polygons[polygon].vertices[point] ] = indices.size();
//          indices.push_back(in_polygons[polygon].vertices[point]);
//        }
//    }
//
//    // in case all points are used, do nothing and return input mesh
//    if ( indices.size() == nr_points )
//    {
//      out_polygons = in_polygons;
//      return;
//    }
//
//    // copy mesh information (and update indices)
//    out_polygons.reserve(nr_polygons);
//    for ( int polygon = 0; polygon < nr_polygons; ++polygon)
//    {
//      pcl::Vertices corrected_polygon;
//      const int nr_points_polygon = (int)in_polygons[polygon].vertices.size();
//      corrected_polygon.vertices.resize(nr_points_polygon);
//      for ( size_t point = 0; point < nr_points_polygon; ++point )
//        corrected_polygon.vertices[point] = new_indices[ in_polygons[polygon].vertices[point] ];
//      out_polygons.push_back(corrected_polygon);
//    }
//  }









//  inline void
//  getVertexVertexList(const pcl::PolygonMesh& mesh, std::vector< std::vector<int> >& vertex_list)
//  {
//    vertex_list.clear();
//    if ( mesh.polygons.size() == 0 )
//      return;
//
//    int nr_points = mesh.cloud.width * mesh.cloud.height;
//    int nr_polygons = mesh.polygons.size();
//    vertex_list.resize(nr_points);
//    for ( int i = 0; i < nr_points; ++i)
//      vertex_list[i].clear();
//
////    for ( unsigned int poly = 0; poly < mesh.polygons.size(); ++poly)
////      for ( unsigned int point = 0; point < mesh.polygons[poly].vertices.size(); ++point)
////        for ( unsigned int neighbor = 0; neighbor < mesh.polygons[poly].vertices.size(); ++neighbor)
////          if (neighbor != point)
////            vertex_list[mesh.polygons[poly].vertices[point]].push_back(mesh.polygons[poly].vertices[neighbor]);
//
////    int last_point_in_polygon = mesh.polygons[poly].vertices.size()
////    for ( unsigned int poly = 0; poly < mesh.polygons.size(); ++poly)
////      for ( unsigned int point = 0; point < mesh.polygons[poly].vertices.size(); ++point)
//
////    for (std::vector<pcl::Vertices>::iterator poly_iter = mesh.polygons.begin(); poly_iter < mesh.polygons.rbegin(); ++poly_iter)
//
////    for (std::vector<pcl::Vertices>::iterator poly_iter = mesh.polygons.begin(); poly_iter < mesh.polygons.end(); ++poly_iter)
////      for (std::vector<int>::iterator point_iter = poly_iter->vertices.begin(); point_iter < poly_iter->vertices.rbegin(); ++point_iter)
//
//    for (int idx_poly = 0; idx_poly < nr_polygons; ++idx_poly)
//    {
//      const int idx_last_point = (int)(mesh.polygons[idx_poly].vertices.size() - 1);
//      for (int idx_point = 0; idx_point < idx_last_point; ++idx_point)
//      {
//        const int point = mesh.polygons[idx_poly].vertices[idx_point];
//        const int next_point = mesh.polygons[idx_poly].vertices[idx_point+1];
//        addNeighbor(vertex_list[point], next_point);
//        addNeighbor(vertex_list[next_point], point);
//      }
//      const int first_point = mesh.polygons[idx_poly].vertices[0];
//      const int last_point = mesh.polygons[idx_poly].vertices[idx_last_point];
//      addNeighbor(vertex_list[last_point], first_point);
//      addNeighbor(vertex_list[first_point], last_point);
//    }
//  }


//  inline void
//  getMeshNeighborhood(const std::vector< std::vector<int> >& initial_vertex_list, std::vector< std::vector<int> >& neighborhoods, const int& neighborhood_size = 1, const bool& include_query_point = false)
//  {
//
//    int nr_points = (int)(initial_vertex_list.size());
//    neighborhoods.resize(nr_points);
//    for (int i = 0; i < nr_points; ++i)
//      neighborhoods[i].clear();
//
//    std::vector<bool> is_in_neighborhood(nr_points, false);
//    std::vector<bool> is_on_queue(nr_points, false);
//
//    for (int i = 0; i < nr_points; ++i)
//    {
//      // add query point (and mark as is_in_neighborhood)
//      if (include_query_point)
//        neighborhoods[i].push_back(i);
//      is_in_neighborhood[i] = true;
//
//      // only use initial neighborhood
//      if (neighborhood_size <= 1)
//      {
//        for (unsigned int j = 0; j < initial_vertex_list[i].size(); ++j)
//        {
//          if (!is_in_neighborhood[initial_vertex_list[i][j]])
//          {
//            neighborhoods[i].push_back(initial_vertex_list[i][j]);
//            is_in_neighborhood[initial_vertex_list[i][j]] = true;
//          }
//        }
//      }
//      else // recursively expand neighborhood for each vertex
//      {
//        int current_size = 0;
//        std::queue< std::pair<int,int> > neighbor_queue;
//        neighbor_queue.push(std::pair<int,int>(current_size, i));
//
//        while(!neighbor_queue.empty())
//        {
//          const std::pair<int,int> vertex = neighbor_queue.front();
//          const int& vertex_distance = vertex.first;
//          const int& vertex_index = vertex.second;
//          neighbor_queue.pop();
//          is_on_queue[vertex_index] = false;
//
//          if(!is_in_neighborhood[vertex_index])
//          {
//            neighborhoods[i].push_back(vertex_index);
//            is_in_neighborhood[vertex_index] = true;
//          }
//
//          if (vertex_distance < neighborhood_size)
//            for (unsigned int j = 0; j < initial_vertex_list[vertex_index].size(); ++j)
//              if (!is_in_neighborhood[initial_vertex_list[vertex_index][j]])
//                if (!is_on_queue[initial_vertex_list[vertex_index][j]])
//                  neighbor_queue.push(std::pair<int,int>((vertex_distance+1),initial_vertex_list[vertex_index][j]));
//        }
//      }
//
//      // mark all points as not being is_in_neighborhood (for this vertex)
//      for (unsigned int j = 0; j < neighborhoods[i].size(); ++j)
//        is_in_neighborhood[neighborhoods[i][j]] = false;
//      is_in_neighborhood[i] = false;
//    }
//
//  }


}


#endif /* PCL_COMMON_NEIGHBORS_FROM_MESH_H_ */
