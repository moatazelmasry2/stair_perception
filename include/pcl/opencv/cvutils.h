/*
 * cvutils.h
 *
 *  Created on: Jul 11, 2012
 *      Author: elmasry
 */

#ifndef CVUTILS_H_
#define CVUTILS_H_

#include <stdio.h>
#include <string>
#include <pcl/features/integral_image_normal.h>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "pcl/common/pcl_commons.h"
#include "pcl/opencv/linesegment2d.h"

#define PCL_NORMAL_MP                           1
#define PCL_NORMAL_INTEGRALIMG                  2

namespace pcl
{
  namespace opencv
  {
    /**
     * calculates intensity from a value.for example a point has z-value 1.5, where as minZ for all points=0.5, maxZ=2 and you'd like to
     * map this value to a color between 0,255
     */
    inline float calculateIntensity (float depth, float min, float max)
    {
      float oldValue = depth;
      float newMax = 255;
      float newMin = 0;
      float val = ( (oldValue - min) / (max - min) * (newMax - newMin)) + newMin;
      return val < newMax ? val : newMax;
    }

    /**
     * calculates intensity for a grey scale image from a given depth, the closer the darker, and the further the brighter
     * new_value = ( (old_value - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min
     * depth is defined between -1:3, while intensity is defined between 0:255
     */
    inline float calculateIntensity (float depth)
    {
      float oldMin = -1;
      float oldMax = 2;
      return calculateIntensity (depth, oldMin, oldMax);
    }

    inline float calculateColorFromNormal (float value)
    {
      float min = 0;
      float max = 1;
      if (value < 0)
      {
        value *= -1;
      }

      float val = calculateIntensity (value, min, max);
      return val < 255 ? val : 255;
    }

    /**
     * calculates color for a rgb image from a given normal, ex normal (0,0,1) corresponds to color (0,0,255)
     * new_value = ( (old_value - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min
     * depth is defined#include <pcl/features/integral_image_normal.h> between -1:3, while intensity is defined between 0:255
     */
    inline void calculateColorFromNormal (pcl::Normal normal, float &r, float &g, float &b)
    {
      r = calculateColorFromNormal (normal.normal_x);
      g = calculateColorFromNormal (normal.normal_y);
      b = calculateColorFromNormal (normal.normal_z);
    }

    /**
     * point cloud must be organized. PointCloud must contain the fields r,g,b
     */
    template<typename PointT>
    void cloudToRGB (const pcl::PointCloud<PointT>& input, cv::Mat& image)
    {
      //image.create (cv::Size (input.width, input.height), CV_8UC3);
      image = cv::Mat::zeros (cv::Size (input.width, input.height), CV_8UC3);
      for (size_t i = 0; i < input.height; i++)
      {
        for (size_t j = 0; j < input.width; j++)
        {
          const PointT& p = input.at (j, i);
          image.at<cv::Vec3b> (i, j)[0] = p.r;
          image.at<cv::Vec3b> (i, j)[1] = p.g;
          image.at<cv::Vec3b> (i, j)[2] = p.b;
        }
      }
    }

    /**
     * Creates a 2D image representing the grayscale range image according
     * to a dimension (default dimension: depth(z) ). The near points are dark and far points are light
     *
     * @compIndex: compare index to compare points according to. default=0, i.e. depth, in world coordinates
     * min, max represents the range in which all points should lie in, i.e. nearest and furthest point. points outside this range will be assigned either white or black
     * if min, max left empty, point cloud will be searched for these values
     */
    template<typename PointT>
    void cloudToRangeGrayscale (const pcl::PointCloud<PointT> cloud, cv::Mat& image, int compIndex = 0, float min = 0,
        float max = 0)
    {
      image = cv::Mat::zeros (cv::Size (cloud.width, cloud.height), CV_8UC1);
      if (min == 0 && max == 0)
      {
        min = 10, max = -10;
        for (size_t i = 0; i < cloud.size (); i++)
        {
          float val = cloud.points[i].getVector3fMap ()[compIndex];
          if (!isnan (val) && val != 0.0f)
          {
            min = val < min ? val : min;
            max = val > max ? val : max;
          }
        }
      }

      printf ("cvUtils:cloudToRangeGrayscale. min=%f, max=%f\n", min, max);
      float intensity = 0;

      int count = 0;
      //last valid zValue;
      for (int i = 0; i < (int) cloud.height; i++)
      {
        for (int j = 0; j < (int) cloud.width; j++)
        {
          float val;
          val = cloud.points[count++].getVector3fMap ()[compIndex];
          if (!isnan (val) && val != 0.0f && val >= min && val <= max)
          {
            intensity = calculateIntensity (val, min, max);
          }
          image.at<uchar> (i, j) = intensity;
        }
      }
    }

    /**
     * converts a point cloud to normals image.pixels are being colored according to the corresponding point normal
     * if removeBorders=true, left right borders will be colored black.because normals at the image borders tend to be messy,
     * and this messes up edges detection later
     */
    template<typename PointT>
    void cloud2Normals (const pcl::PointCloud<PointT> cloud, cv::Mat& image, bool removeBorders = true)
    {
      removeBorders = false;
      image = cv::Mat::zeros (cv::Size (cloud.width, cloud.height), CV_8UC3);
      pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::itnegralImageNormal<PointT> (cloud);

      for (int i = 0; i < (int) cloud.height; i++)
      {
        for (int j = 0; j < (int) cloud.width; j++)
        {
          image.at<cv::Vec3b> (i, j)[0] = 255;
          image.at<cv::Vec3b> (i, j)[1] = 255;
          image.at<cv::Vec3b> (i, j)[2] = 255;
        }
      }

      int counter = 0;
      float r = 0, g = 0, b = 0;
      float tmpR, tmpG, tmpB;
      for (int i = 0; i < (int) cloud.height; i++)
      {
        for (int j = 0; j < (int) cloud.width; j++)
        {
          pcl::Normal& n = normals->points[counter++];

          if (!isnan ( (n.normal_x && !isnan (n.normal_y) && !isnan (n.normal_z))))
          {
            if ( (n.normal_x == 1 && n.normal_y == 1 && n.normal_z == 1))
            {
              continue;
            }
            calculateColorFromNormal (n, tmpR, tmpG, tmpB);
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

          if (removeBorders && ( ((int) j < 30) || (j > ((int) cloud.width - 40))))
          {
            image.at<cv::Vec3b> (i, j)[0] = 255;
            image.at<cv::Vec3b> (i, j)[1] = 255;
            image.at<cv::Vec3b> (i, j)[2] = 255;
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

    template<typename PointT>
    void pointcloud_to_imagenormals (pcl::PointCloud<PointT> cloud, cv::Mat& outImage)
    {
      pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::itnegralImageNormal<PointT> (cloud);
      //IplImage* img = cvCreateImage (cvSize (cloud.width, cloud.height), IPL_DEPTH_32F, 3);
      IplImage* img = cvCreateImage (cvSize (cloud.width, cloud.height), IPL_DEPTH_8U, 3);

      CvScalar s;
      int counter = 0;
      float r = 0, g = 0, b = 0;
      float tmpR, tmpG, tmpB;
      for (int i = 0; i < (int) cloud.height; i++)
      {
        for (int j = 0; j < (int) cloud.width; j++)
        {
          pcl::Normal n = normals->points[counter++];

          if (!isnan ( (n.normal_x && !isnan (n.normal_y) && !isnan (n.normal_z))))
          {
            if ( (n.normal_x == 1 && n.normal_y == 1 && n.normal_z == 1))
            {
              continue;
            }
            calculateColorFromNormal (n, tmpR, tmpG, tmpB);
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
          else
          {
          }
          s = cvGet2D (img, i, j);  // get the (i,j) pixel value
          s.val[0] = r;
          s.val[1] = g;
          s.val[2] = b;
          cvSet2D (img, i, j, s);  // set the (i,j) pixel value
        }
      }
      //return img;
      outImage = img;
    }

    void drawEdges (std::vector<LineSegment2D>& lines, cv::Mat& color_dst)
    {
      for (size_t i = 0; i < lines.size (); i++)
      {
        cv::Vec4i l = lines[i].getLineModel ();
        line (color_dst, cv::Point (l[0], l[1]), cv::Point (l[2], l[3]), cv::Scalar (0, 0, 255), 1, CV_AA);
      }
    }

    void drawEdges (std::vector<cv::Vec4i>& lines, cv::Mat& color_dst)
    {
      for (size_t i = 0; i < lines.size (); i++)
      {
        cv::Vec4i l = lines[i];
        line (color_dst, cv::Point (l[0], l[1]), cv::Point (l[2], l[3]), cv::Scalar (0, 0, 255), 3, CV_AA);
      }
    }

    void houghP (const cv::Mat& src, int low, int high, cv::Mat& cdst, cv::vector<cv::Vec4i>& lines,
        std::vector<LineSegment2D>& vecLines, int threshold = 80, double minLineLength = 50, double maxLineGap = 20,
        bool canny = true)
    {
      cv::Mat dst, src_gray;
      dst = cv::Mat::zeros (cv::Size (src.cols, src.rows), CV_8UC1);
      /// Convert the image to grayscale
      //printf("cv::utils:houghP, dim=%d, type=%d, CV_8UC1=%d\n", src.dims, src.type(), CV_8UC1);
      if (src.type () == CV_8UC1)
      {
        src.copyTo (src_gray);
      }
      else
      {
        cvtColor (src, src_gray, CV_BGR2GRAY);
      }

      //blur( src_gray, dst, cv::Size(3,3) );
      //medianBlur (src_gray, src_gray, 19);
      if (canny)
      {
        Canny (src_gray, dst, low, high, 3);
      }
      else
      {
        src_gray.copyTo (dst);
      }
      cvtColor (dst, cdst, CV_GRAY2BGR);
      HoughLinesP (dst, lines, 1, CV_PI / 180, threshold, minLineLength, maxLineGap);

      for (size_t i = 0; i < lines.size (); i++)
      {
        cv::Vec4i& l = lines[i];
        LineSegment2D line (i);
        line.setLineModel (l);
        vecLines.push_back (line);
      }
      drawEdges (vecLines, cdst);
    }

    /**
     * From a set of 2d lines creates a point cloud holding these lines. depth data are extracted from inputCloud
     * if organized the output point cloud will have the same dimension as the input point cloud, else, only the edges points will be added
     * inputcloud must be organized
     */
    template<typename PointT, typename PointOut>
    void linesToCloud (const typename pcl::PointCloud<PointT>::ConstPtr inputCloud, std::vector<LineSegment2D> lines,
        typename pcl::PointCloud<PointOut>::Ptr outCloud, bool organized = false)
    {
      typedef Eigen::aligned_allocator<cv::Vec2i> Alloc;
      typedef std::vector<cv::Vec2i, Alloc> Line;
      typedef std::vector<Line, Alloc> LineVector;

      float white = pcl::generateColor (255, 254, 254);
      float black = pcl::generateColor (0, 0, 0);

      //inputCloud.width = img.cols;
      //inputCloud.height = img.rows;
      PointOut p;
      //black point
      p.x = NAN;
      p.y = NAN;
      p.z = NAN;
      p.edge = -1;
      p.rgb = black;
      if (organized)
      {
        for (size_t i = 0; i < inputCloud->height; i++)
        {
          for (size_t j = 0; j < inputCloud->width; j++)
          {
            outCloud->push_back (p);
          }
        }
      }

      int tot = 0;
      for (int i = 0; i < (int) lines.size (); i++)
      {
        Line line = lines[i].getPoints ();
        tot += line.size ();
        for (int j = 0; j < (int) line.size (); j++)
        {
          cv::Vec2i cvP = line[j];
          int index = cvP[1] * inputCloud->width + cvP[0];
          const PointT& in = inputCloud->points[index];
          PointOut out;
          out.x = in.x;
          out.y = in.y;
          out.z = in.z;
          out.rgb = white;
          out.edge = i + 1;
          if (organized)
          {
            outCloud->points[index] = out;
          }
          else
          {
            outCloud->push_back (out);
          }
        }
      }
      if (organized)
      {
        outCloud->width = inputCloud->width;
        outCloud->height = inputCloud->height;
      }
      else
      {
        outCloud->width = inputCloud->points.size ();
        outCloud->height = 1;
      }
    }

    void saveImg (const cv::Mat& img, long prefix, std::string suffix)
    {
      char buffer[50];
      sprintf (buffer, "%ld_%s.jpg", prefix, suffix.c_str ());
      cv::imwrite (buffer, img);
    }

    void saveImg (const cv::Mat& img, std::string prefix, long suffix)
    {
      char buffer[50];
      sprintf (buffer, "%s_%ld.jpg", prefix.c_str (), suffix);
      cv::imwrite (buffer, img);
    }

    void saveImg (const cv::Mat& img, std::string outputPath)
    {
      char buffer[50];
      sprintf (buffer, "%s.jpg", outputPath.c_str ());
      cv::imwrite (buffer, img);
    }

    /*
     * Iterates on all pixels and if one has the exact old (r,g,b) replac them with new rgb
     */
    void changeImageColor (cv::Mat& img, int old_r, int old_g, int old_b, int r, int g, int b)
    {
      int width = img.cols;
      int height = img.rows;
      for (int j = 0; j < height; j++)
      {
        for (int k = 0; k < width; k++)
        {
          if (img.at<cv::Vec3b> (j, k)[0] == old_r && img.at<cv::Vec3b> (j, k)[1] == old_g && img.at<cv::Vec3b> (j, k)[2] == old_b)
          {
            img.at<cv::Vec3b> (j, k)[0] = r;
            img.at<cv::Vec3b> (j, k)[1] = g;
            img.at<cv::Vec3b> (j, k)[2] = b;
          }
        }
      }
    }
  }
}
#endif /* CVUTILS_H_ */
