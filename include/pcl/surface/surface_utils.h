/*
 * surface_utils.h
 *
 *  Created on: Feb 25, 2013
 *      Author: elmasry
 */

#ifndef SURFACE_UTILS_H_
#define SURFACE_UTILS_H_

#include <pcl/PolygonMesh.h>
#include <pcl/apps/fast_meshing.h>


namespace pcl
{
  namespace surface
  {
    template<typename PointT, typename PointOut>
    pcl::PolygonMesh::Ptr getMesh (const typename pcl::PointCloud<PointT>::ConstPtr input)
    {
      pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
      pcl::FastMeshing<PointT, PointOut> mesh_builder;
      std::vector<int> original_indices;
      mesh_builder.mesh (input, mesh, original_indices);
      //pcl::io::savePolygonFileVTK ("temp_mesh_raw.vtk", *mesh);
      return mesh;
    }
  }
}

#endif /* SURFACE_UTILS_H_ */
