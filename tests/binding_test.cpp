
#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>


#include <pcl/point_cloud.h>

#include <pcl/filters/extract_indices.h>

typedef pcl::PointMoXYZRGB PointIn;

int main (int argc, char** argv)
{
  pcl::PointCloud < PointIn > cloud;
  cloud.points.resize (2);
  cloud.width = 2;
  cloud.height = 1;

  cloud.points[0].x = cloud.points[0].y = cloud.points[0].z = 0;
  cloud.points[1].x = cloud.points[1].y = cloud.points[1].z = 3;
  pcl::ExtractIndices<PointIn> extract;

}
