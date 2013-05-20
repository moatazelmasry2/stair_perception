/*
 * blackbg_to_white.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: elmasry
 */

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <stdio.h>

int main (int argc, char **argv)
{

  if (argc < 2)
  {
//    fprintf (std::stderr, "Usage: rosrun blackbg_to_white image.png \n");
    std::cout << "error << std::endl";
    return 1;
  }

  cv::Mat image;
  image = cv::imread (argv[1], CV_LOAD_IMAGE_COLOR);

  if (!image.data)                              // Check for invalid input
  {
    std::cout << "Could not open or find the image" << std::endl;
    return -1;
  }

  int threshold = 1;
  for (int i = 0; i < image.rows; i++)
  {
    for (int j = 0; j < image.cols; j++)
    {

      if (image.at<cv::Vec3b> (i, j)[0] <= threshold && image.at<cv::Vec3b> (i, j)[1] <= threshold
          && image.at<cv::Vec3b> (i, j)[2] <= threshold)
      {
        image.at<cv::Vec3b> (i, j)[0] = 255;
        image.at<cv::Vec3b> (i, j)[1] = 255;
        image.at<cv::Vec3b> (i, j)[2] = 255;
      }
    }
  }

  cv::imwrite("newimg.png", image);
}
