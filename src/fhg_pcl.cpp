//#include  "pcl/kdtree/fixed_neighbors.cpp"

#include <pcl/io/impl/pcd_io.hpp>


#include "pcl/filters/extract_indices.h"
#include "pcl/filters/impl/extract_indices.hpp"
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/impl/integral_image_normal.hpp>
#include <pcl/features/impl/normal_3d.hpp>

#include <pcl/kdtree/impl/fixed_neighbors.hpp>

#include <pcl/segmentation/impl/sac_segmentation.hpp>

#include <pcl/filters/impl/extract_indices.hpp>

#include <pcl/registration/impl/correspondence_estimation.hpp>

#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/organized.hpp>

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
#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_stick.hpp>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/impl/rransac.hpp>
