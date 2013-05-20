/*
 * fhg_models.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: elmasry
 */


#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"
#include <pcl/common/fhg_point_types.h>

#include "pcl/models/edges2planesfactory.h"
#include "pcl/models/globalmodel.h"
#include "pcl/models/localmodel.h"
#include "pcl/models/localmodelfactory.h"
#include "pcl/models/model_utils.h"
#include "pcl/models/modelsampler.h"


PCL_INSTANTIATE(Edges2PlanesFactory, (pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(GlobalModel, (pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(LocalModel, (pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(LocalModelFactory, (pcl::PointMoXYZRGBNormal))
PCL_INSTANTIATE(ModelSampler, (pcl::PointMoXYZRGBNormal))
