/*
 * mo_io.h
 *
 *  Created on: Feb 6, 2013
 *      Author: elmasry
 */

#ifndef MO_IO_H_
#define MO_IO_H_

#include <pcl/point_cloud.h>

#include "pcl/models/localmodel.h"
#include "pcl/models/globalmodel.h"

namespace pcl
{
  namespace io
  {
    template<typename PointT>
    void saveModel (const std::string file_name, const LocalModel<PointT>& model);

    template<typename PointT>
        void saveMoPcd (const char* name, pcl::PointCloud<PointT> cloud);

  }
}

#include "pcl/io/impl/mo_io.hpp"

#endif /* MO_IO_H_ */
