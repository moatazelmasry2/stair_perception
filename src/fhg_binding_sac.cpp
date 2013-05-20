/*
 * fhg_binding_sac.cpp
 *
 *  Created on: Feb 14, 2013
 *      Author: elmasry
 */
#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

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

PCL_INSTANTIATE(SampleConsensusModelCircle2D, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(SampleConsensusModelCircle3D, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(SampleConsensusModelPlane, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(SampleConsensusModelPerpendicularPlane, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(SampleConsensusModelParallelPlane, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(SampleConsensusModelLine, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(SampleConsensusModelParallelLine, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(SampleConsensusModelStick, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(SampleConsensusModelSphere, PCL_XYZ_POINT_TYPES PointMoTypes)

PCL_INSTANTIATE(RandomSampleConsensus, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(MEstimatorSampleConsensus, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(RandomizedMEstimatorSampleConsensus, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(RandomizedRandomSampleConsensus, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(ProgressiveSampleConsensus, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(MaximumLikelihoodSampleConsensus, PCL_XYZ_POINT_TYPES PointMoTypes)
PCL_INSTANTIATE(LeastMedianSquares, PCL_XYZ_POINT_TYPES PointMoTypes)




