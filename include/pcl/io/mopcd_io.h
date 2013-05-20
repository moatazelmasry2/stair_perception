/*
 * mopcd_io.h
 *
 *  Created on: Jan 15, 2013
 *      Author: elmasry
 */

#ifndef MOPCD_IO_H_
#define MOPCD_IO_H_

#endif /* MOPCD_IO_H_ */

namespace pcl
{
  namespace io
  {

    /**
     * loads a pcd file and adds an index id to it.
     */
    template<typename PointIn, typename PointOut>
      int
      moloadPCDFile (char *path, pcl::PointCloud<PointOut>& outCloud)
      {

        pcl::PointCloud<PointIn> inCloud;

        if (pcl::io::loadPCDFile<PointIn> (path, inCloud) == -1) return -1;

        pcl::copyPointCloud (inCloud, outCloud);
        for (size_t i = 0; i < outCloud.size (); i++)
        {
          outCloud[i].id = (int)i;
        }
        return 0;

      }
  }
}
