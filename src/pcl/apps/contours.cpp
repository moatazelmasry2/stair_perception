/*
 * contours.cpp
 *
 *  Created on: Jul 22, 2012
 *      Author: elmasry
 */

#include "opencv-2.3.1/opencv2/highgui/highgui.hpp"
#include "opencv-2.3.1/opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "pcl/opencv/cvutils.h"
#include "pcl/opencv/linesegment2d.h"
#include "pcl/opencv/edge_detection_utils.h"
#include "pcl/opencv/imageconcat.h"

using namespace cv;
using namespace std;

Mat src;
Mat src_gray;
int low_thresh = 20;
int high_thresh  = 40;
int hough_thresh = 80;
int hough_minLenght = 50;
int hough_maxGap = 10;
int max_thresh = 100;
RNG rng (12345);

/// Function header
void
thresh_callback (int, void*);

/** @function main */
int
main (int argc, char** argv)
{
  /// Load source image and convert it to gray
  src = imread (argv[1], 1);
  string s = argv[1];
  int index = s.find_last_of ("/", s.size () - 1);
  s = s.substr (index + 1, s.length () - index - 4);
  string path = s;

  /// Create Window
  char* source_window = "Source";
  namedWindow (source_window, CV_WINDOW_AUTOSIZE);
  imshow (source_window, src);

  createTrackbar ("Canny lowthresh:", "Source", &low_thresh, max_thresh, thresh_callback);
  createTrackbar ("Canny highthresh:", "Source", &high_thresh, max_thresh, thresh_callback);
  createTrackbar ("hough thresh:", "Source", &hough_thresh, 200, thresh_callback);
  createTrackbar ("hough minlength:", "Source", &hough_minLenght, 200, thresh_callback);
  createTrackbar ("hough maxGap:", "Source", &hough_maxGap, 50, thresh_callback);
  thresh_callback (0, 0);

  waitKey (0);
  return (0);
}

void
thresh_callback (int, void*)
{
  typedef std::vector<pcl::LineSegment2D> LineSegment2DVector;
  LineSegment2DVector vecLines;

  pcl::opencv::ImageConcatenator conc;
  //imwrite (path.assign (s.c_str ()).append ("_a.jpg"), src);
  conc.addImage (src);
  /// Convert image to gray and blur it
  cvtColor (src, src_gray, CV_BGR2GRAY);
  blur (src_gray, src_gray, Size (3, 3));

  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny (src_gray, canny_output, low_thresh, high_thresh, 3);
  namedWindow ("canny", CV_WINDOW_AUTOSIZE);
  imshow("canny", canny_output);
  /// Find contours
  //findContours (canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point (0, 0));
  Mat drawing = Mat::zeros (canny_output.size (), CV_8UC3);
  //canny_output.copyTo(drawing);
  /*for (size_t i = 0; i < contours.size (); i++)
  {
    Scalar color = Scalar (255, 255, 255);
    drawContours (drawing, contours, i, color, 2, 8, hierarchy, 0, Point ());
  }*/
  conc.addImage (drawing);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  cv::vector<cv::Vec4i> lines;
  //src_gray = Mat::zeros(drawing.rows, drawing.cols, CV_8UC1);
  //cvtColor (drawing, src_gray, CV_BGR2GRAY);
  canny_output.copyTo(src_gray);
  HoughLinesP (src_gray, lines, 1, //rho
               CV_PI / 180, //theta
               hough_thresh, //threshold
               hough_minLenght, //minLineLength
               hough_maxGap); //maxLineGap

  for (size_t i = 0; i < lines.size (); i++)
  {
    cv::Vec4i& l = lines[i];
    pcl::LineSegment2D line (i);
    line.setLineModel (l);
    vecLines.push_back (line);
  }
  pcl::opencv::drawEdges (vecLines, drawing);
  conc.addImage (drawing);
  //conc.saveImg (path.assign (s.c_str ()).append ("_a.jpg"), 0);
  namedWindow ("Contours", CV_WINDOW_AUTOSIZE);
  imshow ("Contours", drawing);

}
