/*
 * imageconcat.h
 *
 *  Created on: Jul 14, 2012
 *      Author: elmasry
 *
 *      Takes a group of images, concat them together and produces one output image
 */

#ifndef IMAGECONCAT_H_
#define IMAGECONCAT_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "pcl/opencv/cvutils.h"

namespace pcl
{
  namespace opencv
  {
    class ImageConcatenator
    {
        std::vector<cv::Mat> images;

      public:
        int cols;

        ImageConcatenator () :
            cols (4)
        {

        }
        /**
         * TODO make sure that all inserted images have same width and height
         */
        inline void addImage (const cv::Mat& img)
        {
          cv::Mat mat = cv::Mat::zeros (cv::Size (img.rows, img.cols), CV_8UC3);
          if (img.type () == CV_8UC1)
          {
            cvtColor (img, mat, CV_GRAY2BGR);
            images.push_back (mat);
          }
          else
          {
            img.copyTo (mat);
            images.push_back (mat);
          }

        }

        inline void saveImg (std::string prefix, long suffix)
        {
          int column = 0;
          int width, height;
          if (images.size () > 0)
          {
            //TODO make sure all images have same width and height, else exit
            cv::Mat& img = images[0];
            width = img.cols;
            height = img.rows;
          }
          else
            return;

          cv::Mat dst = cv::Mat::zeros (cv::Size (width * cols, height * ( (images.size () / cols) + 1)), CV_8UC3);
          cv::Mat roi;
          int row = 0;
          int c = 'a';

          for (size_t i = 0; i < images.size (); i++)
          {
            if (i % cols == 0 && i != 0)
            {
              column = 0;
              row++;
            }
            roi = dst (cv::Rect (column * width, row * height, width, height));
            column++;

            images[i].copyTo (roi);
            char text[10];
            sprintf (text, "%c)", (int) c++);
            cv::putText (roi, text, cv::Point (10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar (255, 255));
          }

          pcl::opencv::saveImg (dst, prefix, suffix);
        }

        inline void saveSingleImages (std::string prefix)
        {
          for (size_t i = 0; i < images.size (); i++)
          {
            cv::Mat& img = images[i];
            int width = img.cols;
            int height = img.rows;

            for (int j = 0; j < height; j++)
            {
              for (int k = 0; k < width; k++)
              {
                if (img.at<cv::Vec3b> (j, k)[0] == 0 && img.at<cv::Vec3b> (j, k)[1] == 0
                    && img.at<cv::Vec3b> (j, k)[2] == 0)
                {
                  img.at<cv::Vec3b> (j, k)[0] = 255;
                  img.at<cv::Vec3b> (j, k)[1] = 255;
                  img.at<cv::Vec3b> (j, k)[2] = 255;
                }
              }
            }
            char text[20];
            sprintf (text, "%s_%i.png", prefix.c_str (), (int) i);
            cv::imwrite (text, img);
          }
        }
    };

  }
}

#endif /* IMAGECONCAT_H_ */
