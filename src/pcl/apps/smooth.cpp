#include "opencv-2.3.1/opencv2/imgproc/imgproc.hpp"
#include "opencv-2.3.1/opencv2/highgui/highgui.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"

#include "pcl/opencv/cvutils.h"
#include "pcl/opencv/linesegment2d.h"
#include "pcl/opencv/edge_detection_utils.h"

using namespace std;
using namespace cv;

/// Global Variables
int MAX_KERNEL_LENGTH = 31;

Mat src;
Mat dst, cdst, mat_hom, mat_gauss, mat_median, mat_bi;
char window_name[] = "Filter Demo 1";

/// Function headers

double
getTime ()
{
  using namespace boost::posix_time;
  using namespace boost::gregorian;
  ptime time_t_epoch (date (1970, 1, 1));
  ptime now = microsec_clock::local_time ();
  time_duration diff = now - time_t_epoch;
  long x = diff.total_milliseconds ();
  return x;
}
/**
 * function main
 */
int
main (int argc, char** argv)
{
  typedef std::vector<pcl::LineSegment2D> LineSegment2DVector;
  //namedWindow( window_name, CV_WINDOW_AUTOSIZE );

  /// Load the source image
  src = imread (argv[1], 1);
  int kernel = 31;
  int method = 1;
  int low = 0, high = 7, kernel_canny = 3;
  LineSegment2DVector vecLines;

  if (argc > 2)
  {
    method = atoi (argv[2]);
  }

  if (argc > 3)
  {
    kernel = atoi (argv[3]);
  }

  if (argc > 4)
  {
    low = atoi (argv[4]);
  }

  if (argc > 5)
  {
    high = atoi (argv[5]);
  }

  if (argc > 6)
  {
    kernel_canny = atoi (argv[6]);
  }
  string s = argv[1];

  int index = s.find_last_of ("/", s.size () - 1);
  s = s.substr (index + 1, s.length () - index - 4);
  string path = s;

  //blur (src, mat_hom, Size (kernel, kernel), Point (-1, -1));
  //GaussianBlur (src, mat_gauss, Size (kernel, kernel), 0, 0);
  //medianBlur (src, mat_median, kernel);
  //bilateralFilter (src, mat_bi, kernel, kernel * 2, kernel / 2);
  printf("method=%d, kernel=%d, low=%d, high=%d, kernelcanny=%d\n", method, kernel, low, high, kernel_canny);
  cv::Mat dst_f;
  switch (method)
  {
    case 1:
      blur (src, dst_f, Size (kernel, kernel), Point (-1, -1));
      break;
    case 2:
      GaussianBlur (src, dst_f, Size (kernel, kernel), 0, 0);
      break;
    case 3:
      medianBlur (src, dst_f, kernel);
      break;
    case 4:
      bilateralFilter (src, dst_f, kernel, kernel * 2, kernel / 2);
      break;
    default:
      dst_f = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);
      src.copyTo(dst_f);
  }

  //imwrite (path.assign (s.c_str ()).append ("_mat_hom.jpg"), mat_hom);
  //imwrite (path.assign (s.c_str ()).append ("_mat_gauss.jpg"), mat_gauss);
  //imwrite (path.assign (s.c_str ()).append ("_mat_median.jpg"), mat_median);
  //imwrite (path.assign (s.c_str ()).append ("_mat_bi.jpg"), mat_bi);
  imwrite (path.assign (s.c_str ()).append ("_mat.jpg"), dst_f);

  cv::Mat dst, src_gray;
  dst = cv::Mat::zeros (cv::Size (src.cols, src.rows), CV_8UC1);
  cvtColor (dst_f, src_gray, CV_BGR2GRAY);
  //cvtColor (mat_median, src_gray, CV_BGR2GRAY);
  //cvtColor (mat_bi, src_gray, CV_BGR2GRAY);
  Canny (src_gray, dst, low, high, kernel_canny);
  //pcl::opencv::saveImg(mat_bi, num, "mat_bi.jpg");
  //imwrite ("canny.jpg", dst);

  cv::vector<cv::Vec4i> lines;
  cvtColor (dst, cdst, CV_GRAY2BGR);
  HoughLinesP (dst, lines,
               1,               //rho
               CV_PI / 180,     //theta
               80,             //threshold
               50,              //minLineLength
               20);             //maxLineGap
  /**
   * HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );

with the arguments:

    dst: Output of the edge detector. It should be a grayscale image (although in fact it is a binary one)
    lines: A vector that will store the parameters (x_{start}, y_{start}, x_{end}, y_{end}) of the detected lines
    rho : The resolution of the parameter r in pixels. We use 1 pixel.
    theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
    threshold: The minimum number of intersections to “detect” a line
    minLinLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    maxLineGap: The maximum gap between two points to be considered in the same line.
   *
   */
  /*for (size_t i = 0; i < lines.size (); i++)
   {
   Vec4i l = lines[i];
   line (cdst, Point (l[0], l[1]), Point (l[2], l[3]), Scalar (0, 0, 255), 3, CV_AA);
   }*/
  for (size_t i = 0; i < lines.size (); i++)
  {
    cv::Vec4i& l = lines[i];
    pcl::LineSegment2D line (i);
    line.setLineModel (l);
    vecLines.push_back (line);
  }
  pcl::opencv::drawEdges (vecLines, cdst);
  imwrite (path.assign (s.c_str ()).append ("_a.jpg"), cdst);
  //pcl::opencv::removeVerticalLines (vecLines);
  cdst = cv::Mat::zeros (cdst.rows, cdst.cols, CV_8UC3);
  pcl::opencv::drawEdges (vecLines, cdst);
  imwrite (path.assign (s.c_str ()).append ("_f.jpg"), cdst);
  printf ("done\n");

  return 0;
}
