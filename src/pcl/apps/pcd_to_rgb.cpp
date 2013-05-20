/*
 * pcd_to_rgb.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: elmasry
 */

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "pcl/opencv/cvutils.h"

typedef pcl::PointXYZRGB PointIn;
int
main (int argc, char **argv)
{

  pcl::PointCloud<PointIn>::Ptr cloud (new pcl::PointCloud<PointIn>);

  if (argc < 2)
  {
    fprintf (stderr, "Usage: rosrun fhg_pcl pcd_to_rgb input.pcd outputfile (optional) \n");
    return 1;
  }

  if (pcl::io::loadPCDFile<PointIn> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s\n", argv[1]);
    return -1;
  }

  std::string outputfile;
  if (argc >= 3) {
    outputfile = argv[2];
  } else {
    std::string inFile = argv[1];
    size_t pos = inFile.find_last_of('/');
    if (pos != std::string::npos) {
      outputfile = inFile.substr(pos+1);
    }
  }

  cv::Mat image;
  pcl::opencv::cloudToRGB(*cloud, image);
  pcl::opencv::saveImg(image, outputfile);

}
