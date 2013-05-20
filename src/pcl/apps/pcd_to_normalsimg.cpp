/*
 * pcd_to_imagenormals.cpp
 *
 *  Created on: May 23, 2012
 *      Author: elmasry
 *
 *   Takes in a pcd file, extracts normals using pcl::IntegralImageNormalEstimation, and builds rgb image
 */

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

#include <opencv-2.3.1/opencv2/opencv.hpp>
#include <opencv-2.3.1/opencv2/highgui/highgui.hpp>

#include "pcl/common/motime.h"
#include "pcl/common/pcl_commons.h"
#include "pcl/opencv/cvutils.h"

template<typename PointT>
  pcl::PointCloud<pcl::Normal>::Ptr
  itnegralImageNormal (typename pcl::PointCloud<PointT>::ConstPtr cloud, float depthChangeFactor = 0.02f, float normalSmoothSize = 10.0f)
  {
    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor (depthChangeFactor);
    ne.setNormalSmoothingSize (normalSmoothSize);
    ne.setInputCloud (cloud);
    ne.compute (*normals);
    return normals;
  }

/**
 * converts a point cloud to normals image.pixels are being colored according to the corresponding point normal
 * if removeBorders=true, left right borders will be colored black.because normals at the image borders tend to be messy,
 * and this messes up edges detection later
 */
template<typename PointT>
  void
  cloud2Normals (const pcl::PointCloud<PointT> cloud, cv::Mat& image, bool removeBorders = true, float depthChangeFactor = 0.02f, float normalSmoothSize = 10.0f)
  {
    image = cv::Mat::zeros (cv::Size (cloud.width, cloud.height), CV_8UC3);
    pcl::PointCloud<pcl::Normal>::Ptr normals = itnegralImageNormal<PointT> (cloud.makeShared (), depthChangeFactor, normalSmoothSize);
    //pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::common::dirksNormals<PointT> (cloud.makeShared ());
    //pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::common::estimateNormals<PointT> (cloud.makeShared ());
    //pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::common::fastOrganizedMesh<PointT> (cloud.makeShared ());
    //pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::common::estimateNormalsOMP<PointT> (cloud.makeShared ());

    int counter = 0;
    float r = 0, g = 0, b = 0;
    float tmpR, tmpG, tmpB;
    for (int i = 0; i < (int)cloud.height; i++)
    {
      for (int j = 0; j < (int)cloud.width; j++)
      {
        pcl::Normal& n = normals->points[counter++];

        if (!isnan ((n.normal_x && !isnan (n.normal_y) && !isnan (n.normal_z))))
        {
          if ((n.normal_x == 1 && n.normal_y == 1 && n.normal_z == 1))
          {
            continue;
          }
          pcl::opencv::calculateColorFromNormal (n, tmpR, tmpG, tmpB);
          if (fabs (255 - tmpR) < 5 && fabs (255 - tmpG) < 5 && fabs (255 - tmpB) < 5)
          {

          }
          else
          {
            r = tmpR;
            g = tmpG;
            b = tmpB;
          }
        }

        if (removeBorders && (((int)j < 30) || (j > ((int)cloud.width - 40))))
        {
          image.at<cv::Vec3b> (i, j)[0] = 0;
          image.at<cv::Vec3b> (i, j)[1] = 0;
          image.at<cv::Vec3b> (i, j)[2] = 0;
        }
        else
        {
          image.at<cv::Vec3b> (i, j)[0] = r;
          image.at<cv::Vec3b> (i, j)[1] = g;
          image.at<cv::Vec3b> (i, j)[2] = b;
        }

      }
    }
  }

int
main (int argc, char **argv)
{

  float depthChangeFactor = 0.02f, normalSmoothSize = 10.0f;
  typedef pcl::PointXYZRGB PointIn;
  pcl::PointCloud<PointIn>::Ptr cloud (new pcl::PointCloud<PointIn>);

  if (argc < 2)
  {
    fprintf (stderr, "Usage: rosrun fhg_pcl pcd_to_greyscalerange input.jpg outprefix (optional) \n");
    return 1;
  }

  if (pcl::io::loadPCDFile<PointIn> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s\n", argv[1]);
    return -1;
  }

  if (argc > 2)
  {
    depthChangeFactor = atof (argv[2]);
  }

  if (argc > 3)
  {
    normalSmoothSize = atof (argv[3]);
  }

  cv::Mat img;
  cloud2Normals (*cloud, img, false, depthChangeFactor, normalSmoothSize);
  pcl::opencv::saveImg (img, pcl::getTimeMs (), "normal");
  return 0;
}

