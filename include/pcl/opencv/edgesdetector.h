/*
 * edgesdetector.h
 *
 *  Created on: Jul 11, 2012
 *      Author: elmasry
 */

#ifndef EDGESDETECTOR_H_
#define EDGESDETECTOR_H_

#include <map>

#include <pcl/point_cloud.h>
#include "pcl/pcl_base.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "pcl/sample_consensus/method_types.h"

#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/sac_model_line.h"

#include "pcl/histogram/linesegmenthistogram.h"
#include "pcl/common/fhg_point_types.h"
#include "pcl/opencv/cvutils.h"
#include "pcl/opencv/linesegment2d.h"
#include "pcl/opencv/linesegment2dhistogram.h"
#include "pcl/common/color.h"
#include "pcl/common/pcl_commons.h"
#include "pcl/utils/pointcloud_utils.h"
#include "pcl/common/motime.h"
#include "pcl/io/pcd_io.h"
#include "pcl/utils/pointcloud_utils.h"
#include "pcl/opencv/edge_detection_utils.h"
#include "pcl/common/time.h"
#include "pcl/common/point_common.h"
#include "pcl/opencv/imageconcat.h"
#include "pcl/common/linesegment2d_common.h"
#include "pcl/histogram/linesegment3dhistogram.h"
#include "pcl/common/edges_common.h"
#include "pcl/io/mo_io.h"

namespace pcl
{
  namespace opencv
  {
    /**
     * PointIn has to be of type PointXYZRGB
     */
    template<typename PointIn, typename PointOut>
    class EdgeDetector : public pcl::PCLBase<PointIn>
    {

        std::string edgesfile;
      protected:

        typedef std::vector<LineSegment2D> LineSegment2DVector;
        typedef pcl::PointCloud<PointIn> PointCloud;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef std::vector<pcl::LineSegment3D<PointOut>, Eigen::aligned_allocator<PointOut> > LineSegment3DVector;
        typedef std::vector<ColumnBin> ColumnBins;
        typedef std::vector<RowBin> BinsMatrix;

        LineSegment3DVector verticalLines;
        pcl::opencv::ImageConcatenator conc;

      protected:

        void extractEdgesContours (const cv::Mat& rgbsrc, const cv::Mat& src, LineSegment3DVector& linesVector, int low,
            int high, int threshold = 80, double minLineLength = 50, double maxLineGap = 20)
        {
          typedef std::vector<pcl::LineSegment2D> LineSegment2DVector;
          LineSegment2DVector vecLines;
          cv::Mat src_gray;
          cvtColor (src, src_gray, CV_BGR2GRAY);
          blur (src_gray, src_gray, cv::Size (3, 3));

          cv::Mat tmpMat = cv::Mat::zeros (cv::Size (src.cols, src.rows), CV_8UC3);
          conc.addImage (src);
          cv::Mat canny_output;
          vector<vector<cv::Point> > contours;
          vector<cv::Vec4i> hierarchy;

          /// Detect edges using canny
          Canny (src_gray, canny_output, low, low * 2, 3);

          rgbsrc.copyTo (tmpMat);
          cvtColor (canny_output, tmpMat, CV_GRAY2RGB);
          conc.addImage(tmpMat);
          pcl::opencv::changeImageColor (tmpMat, 255, 255, 255, 170, 150, 150);
          conc.addImage (tmpMat);
          /// Find contours
          findContours (canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point (0, 0));

          /// Draw contours
          cv::Mat drawing = cv::Mat::zeros (canny_output.size (), CV_8UC3);
//          rgbsrc.copyTo(drawing);
          rgbsrc.copyTo (tmpMat);
          for (size_t i = 0; i < contours.size (); i++)
          {
            //red_grey
            cv::Scalar color = cv::Scalar (170, 150, 150);
            drawContours (drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point ());
            drawContours (tmpMat, contours, i, color, 2, 8, hierarchy, 0, cv::Point ());
          }

//          conc.addImage (drawing);
          conc.addImage (tmpMat);
          /////////////////////////////////////////////////////////////////////////////////////////////////////////
          /////////////////////////////////////////////////////////////////////////////////////////////////////////
          /////////////////////////////////////////////////////////////////////////////////////////////////////////
          vector<cv::Vec4i> lines;
          cvtColor (drawing, src_gray, CV_BGR2GRAY);
          HoughLinesP (src_gray, lines, 1, CV_PI / 180, threshold, minLineLength, maxLineGap);

          for (size_t i = 0; i < lines.size (); i++)
          {
            cv::Vec4i& l = lines[i];
            pcl::LineSegment2D line (i);
            line.setLineModel (l);
            //printf("%f, ", line.getThetaDegree());
            vecLines.push_back (line);
          }
//          rgbsrc.copyTo(tmpMat);
//          drawing.copyTo (tmpMat);
          pcl::opencv::drawEdges (vecLines, tmpMat);
          conc.addImage (tmpMat);

          removeUpperLines (vecLines, input_->height / 4);
          /// after hough lines
          pcl::opencv::drawEdges (vecLines, drawing);
          //printf ("houghLines=%d", (int)vecLines.size ());
          conc.addImage (drawing);

          rgbsrc.copyTo (tmpMat);
          pcl::opencv::drawEdges (vecLines, tmpMat);
          conc.addImage (tmpMat);
          //post process lines after hough
          pcl::opencv::removeShortLines (vecLines, input_->width / 3);
          rgbsrc.copyTo (tmpMat);
          pcl::opencv::drawEdges (vecLines, tmpMat);
          conc.addImage (tmpMat);
          //pcl::opencv::removeBorderLines (vecLines, input_->width, input_->height, 10);
          pcl::opencv::removeVerticalLines (vecLines);
          //draw lines after post process
          rgbsrc.copyTo (tmpMat);
          pcl::opencv::drawEdges (vecLines, tmpMat);
          conc.addImage (tmpMat);
          //printf (", postProcess=%d", (int)vecLines.size ());

          pcl::LineSegment2DHistogram hist;
          for (size_t i = 0; i < vecLines.size (); i++)
          {
            hist.addLine (vecLines[i]);
          }
          hist.init ();
          hist.calculate ();
          //DEBUG draw colored lines before and after merge
//          tmpMat = cv::Mat::zeros (cv::Size (input_->width, input_->height), CV_8UC3);
          rgbsrc.copyTo (tmpMat);
          drawColoredEdges (hist.getOldMatrix (), tmpMat);
          conc.addImage (tmpMat);
//          tmpMat = cv::Mat::zeros (cv::Size (input_->width, input_->height), CV_8UC3);
          rgbsrc.copyTo (tmpMat);
          drawColoredEdges (hist.getMatrix (), tmpMat);
          conc.addImage (tmpMat);
          LineSegment2DVector tmpLines;
          hist.getLines (tmpLines);
          //printf (", afterHistogram=%d", (int)tmpLines.size ());
          ///////////////       END DEBUG  /////////////////////////////////////
          hist.migrateNeighboringBins2 ();
          /////////////////////  DEBUG   ////////////////////////////////////////////
//          tmpMat = cv::Mat::zeros (cv::Size (input_->width, input_->height), CV_8UC3);
          rgbsrc.copyTo (tmpMat);
          drawColoredEdges (hist.getMatrix (), tmpMat);
          conc.addImage (tmpMat);

          /////////////////////  END DEBUG   ////////////////////////////////////////////
          LineSegment2DVector lineSegments;
          hist.getLines (lineSegments);
          //printf (", afterMigrateBins=%d", (int)lineSegments.size ());
          filterBadLines (lineSegments, lineSegments, hist.getDominantOrientation (), input_->width / 8, 8);
          //printf (", afterFilter=%d", (int)lineSegments.size ());
          removeIntersectingLines (lineSegments, hist.getDominantOrientation ());
          //printf (", remvIntersection=%d", (int)lineSegments.size ());
          //pcl::common::logLines(lineSegments);
          pcl::common::mergeNeighboringLines (lineSegments, hist.getDominantOrientation ());
          //pcl::common::logLines(lineSegments);
          //printf (", mergeNeighboringLines=%d", (int)lineSegments.size ());
          /////////////////////  DEBUG   ////////////////////////////////////////////
//          tmpMat = cv::Mat::zeros (cv::Size (input_->width, input_->height), CV_8UC3);
          rgbsrc.copyTo (tmpMat);
          drawEdges (lineSegments, tmpMat);
          conc.addImage (tmpMat);
          /////////////////////  END DEBUG   ////////////////////////////////////////////
//          tmpMat = cv::Mat::zeros (cv::Size (src.cols, src.rows), CV_8UC3);
          rgbsrc.copyTo (tmpMat);
          pcl::opencv::drawEdges (lineSegments, tmpMat);

//          tmpMat = cv::Mat::zeros (cv::Size (src.cols, src.rows), CV_8UC3);
          rgbsrc.copyTo (tmpMat);
          pcl::opencv::drawEdges (lineSegments, tmpMat);
          conc.addImage (tmpMat);
          /////////////////////DONE WITH 2D. DO THE 3D WORK  /////////////////////////
          ///////////////////////////////////////////////////////////////////////////////////
          std::map<int, pcl::PointCloud<PointOut> > lineMap;
          typename pcl::PointCloud<PointOut>::Ptr inCloud (new pcl::PointCloud<PointOut>);
          pcl::copyPointCloud (*input_, *inCloud);
          linesVector = projectEdges (inCloud, lineSegments);
          //long num = pcl::getTimeMs ();
//          conc.saveImg ("a_", num);
//          conc.saveSingleImages (std::string ("image"));

        }

        std::vector<pcl::LineSegment3D<PointOut>, Eigen::aligned_allocator<PointOut> > projectEdges (
            const typename pcl::PointCloud<PointOut>::Ptr inputCloud, std::vector<LineSegment2D> lines)
        {
          std::vector<pcl::LineSegment3D<PointOut>, Eigen::aligned_allocator<PointOut> > outLines;
          typedef std::vector<cv::Vec2i, Eigen::aligned_allocator<cv::Vec2i> > LinePoints;
          int counter = 0;
          for (size_t i = 0; i < lines.size (); i++)
          {
            LinePoints points = lines[i].getPoints ();
            typename pcl::PointCloud<PointOut>::Ptr inCloud (new pcl::PointCloud<PointOut>);
            for (size_t j = 0; j < points.size (); j++)
            {
              cv::Vec2i cvP = points[j];
              int index = cvP[1] * inputCloud->width + cvP[0];
              const PointOut& in = inputCloud->points[index];
              inCloud->push_back (in);
            }
            if (inCloud->size () < 2)
              continue;
            LineSegment3D<PointOut> linesegment3D (counter++);
            linesegment3D.distance_threshold = 0.08;
            linesegment3D.max_iterations = 500;
            linesegment3D.setInputCloud (inCloud);
            outLines.push_back (linesegment3D);
          }

          return outLines;
        }

        /**
         * Removes lines that are too short or far from the dominantorientation
         *
         * \pram dominantOrientation is in radian
         * \param thresholdDomOrientation. How far max can the line be rotated from the dominant orientation
         */
        void filterBadLines (LineSegment2DVector& input, LineSegment2DVector& output, float dominantOrientation,
            float minLineLength, float thresholdDomOrientation = 8.0f)
        {
          //float minLineLength = length > input_->width / 8
          float dom = pcl::radianToAngle (dominantOrientation);
          LineSegment2DVector tmpLines;
          for (size_t i = 0; i < input.size (); i++)
          {
            LineSegment2D& line1 = input[i];
            //cv::Vec4i model = line1.getLineModel ();

            float angle = fabs (line1.getThetaDegree () - dom);
            //float length = pcl::distance2D (cv::Vec2i (model[0], model[1]), cv::Vec2i (model[2], model[3]));
            //if (angle < 8 && length > input_->width / 8)
            if (line1.isHorizontal () && angle < thresholdDomOrientation && minLineLength)
            {
              tmpLines.push_back (line1);
            }
          }

          output.clear ();
          for (size_t i = 0; i < tmpLines.size (); i++)
          {
            output.push_back (tmpLines[i]);
          }
        }

        void drawEdges (LineSegment2DVector& lines, cv::Mat& color_dst)
        {
          for (size_t i = 0; i < lines.size (); i++)
          {
            cv::Vec4i l = lines[i].getLineModel ();
            line (color_dst, cv::Point (l[0], l[1]), cv::Point (l[2], l[3]), cv::Scalar (0, 0, 255), 1, CV_AA);
          }
        }

        void drawColoredEdges (BinsMatrix bins, cv::Mat& color_dst)
        {
          CvFont font;
          double hScale = 1.0;
          double vScale = 1.0;
          int lineWidth = 1;
          cvInitFont (&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);

          for (size_t i = 0; i < bins.size (); i++)
          {
            ColumnBins colBins = bins[i].colBins;
            for (size_t j = 0; j < colBins.size (); j++)
            {
              LineSegment2DVector lines = colBins[j].lines;
              int r = rand () % 255;
              int g = rand () % 255;
              int b = rand () % 255;

              for (size_t k = 0; k < lines.size (); k++)
              {
                LineSegment2D& lineSeg = lines[k];
                cv::Vec4i l = lineSeg.getLineModel ();
                line (color_dst, cv::Point (l[0], l[1]), cv::Point (l[2], l[3]), cv::Scalar (r, g, b), 1, CV_AA);
              }
              for (size_t k = 0; k < lines.size (); k++)
              {
                LineSegment2D& lineSeg = lines[k];
                char buf[50];
                sprintf (buf, "%d", lineSeg.getId ());
                //cv::Vec4i l = lineSeg.getLineModel ();
                //cv::putText (color_dst, buf, cvPoint (l[0] + rand()%20, l[1]), FONT_HERSHEY_SIMPLEX, 0.6, cvScalar (255, 255, 255));
              }
            }
          }
        }

        /**
         * Noticing that using integral image normal that the far lines are distorted, so we remove them from the result
         * \param distBorder distance of the line to the upper border
         */
        void removeUpperLines (LineSegment2DVector& lines, float borderThreshold)
        {
          LineSegment2DVector tmpLines;
          for (size_t i = 0; i < lines.size (); i++)
          {
            LineSegment2D& line1 = lines[i];
            if (line1.getR () >= borderThreshold)
            {
              tmpLines.push_back (line1);
            }
          }
          lines.clear ();
          lines.assign (tmpLines.begin (), tmpLines.end ());
        }

        /**
         * iterates through all lines, if two lines intersect, remove the one furthest from the dominant angle
         * \param lines input lines
         * \param dominantAngle to check againest, in radian
         */
        void removeIntersectingLines (LineSegment2DVector& lines, float dominantAngle)
        {
          float dom = pcl::radianToAngle (dominantAngle);
          std::vector<int> ledger (lines.size (), 1);
          for (size_t i = 0; i < lines.size (); i++)
          {
            if (ledger[i] < 0)
              continue;
            LineSegment2D& line1 = lines[i];
            cv::Vec4i model1 = line1.getLineModel ();
            for (size_t j = i + 1; j < lines.size (); j++)
            {
              if (i == j)
                continue;
              if (ledger[j] < 0)
                continue;
              LineSegment2D& line2 = lines[j];
              cv::Vec4i model2 = line2.getLineModel ();
              Eigen::Vector2f i0 (0, 0), i1 (0, 0);
              int out = pcl::common::intersect2D_Segments (Eigen::Vector2f (model1[0], model1[1]),
                  Eigen::Vector2f (model1[2], model1[3]), Eigen::Vector2f (model2[0], model2[1]),
                  Eigen::Vector2f (model2[2], model2[3]), i0, i1);

              //printf ("intersect.line%d, line%d\n", line1.getId (), line2.getId ());
              if (out == 1 || out == 2)
              {
                if (fabs (dom - line1.getThetaDegree ()) < fabs (dom - line2.getThetaDegree ()))
                {
                  //remove line2
                  ledger[j] = -1;
                }
                else
                {
                  //remove line1
                  ledger[i] = -1;
                  break;
                }
              }
            }
          }                //end primary for loop
          LineSegment2DVector tmpLines;
          for (size_t i = 0; i < lines.size (); i++)
          {
            if (ledger[i] > 0)
            {
              tmpLines.push_back (lines[i]);
            }
          }
          lines.clear ();
          lines.assign (tmpLines.begin (), tmpLines.end ());
        }

        /**
         * find the jump in depth between neighboring points, and find points whose neighbors are far
         * \param component to compare, 0,1,2 corresponds to x,y,z
         * \param distThresold if distance between point and its neighbors is larger than this thresold, then the point is added to the output
         * return a matrix, where valid points are marked 1, while invalid as -1
         */

        void cloud2DepthJumpMatrix (const pcl::PointCloud<PointIn>& cloud, int component, float distThreshold,
            std::vector<std::vector<int> >& indices, typename pcl::PointCloud<PointIn>::Ptr outcloud)
        {
          indices.clear ();
          for (size_t i = 0; i < cloud.height; i++)
          {
            std::vector<int> col (cloud.width, -1);
            indices.push_back (col);
          }

          //for (size_t i = cloud.height - 1; i >= 0; i--)
          for (size_t i = 0; i < cloud.height; i++)
          {
            for (size_t j = 0; j < cloud.width; j++)
            {
              const PointIn& cur = cloud (j, i);
              bool added = false;
              if ( (j + 1 < cloud.width && distCompPoint (cur, cloud (j + 1, i), 2) > distThreshold)
                  || (j + 1 == cloud.width))
              {
                added = true;
                if (indices[i][j + 1] != 1)
                {
                  indices[i][j + 1] = 1;
                  outcloud->push_back (cloud (j + 1, i));
                }

              }

              if ( (i + 1 < cloud.height && distCompPoint (cur, cloud (j, i + 1), 2) > distThreshold))
              {
                added = true;
                if (indices[i + 1][j] != 1)
                {
                  indices[i + 1][j] = 1;
                  outcloud->push_back (cloud (j, i + 1));
                }
              }
              if ( (j + 1 < cloud.width && i + 1 < cloud.height
                  && distCompPoint (cur, cloud (j + 1, i + 1), 2) > distThreshold))
              {
                added = true;
                if (indices[i + 1][j + 1] != 1)
                {
                  indices[i + 1][j + 1] = 1;
                  outcloud->push_back (cloud (j + 1, i + 1));
                }
              }

              //add border points
              if ( (j + 1 < cloud.width && pcl::isNanPoint (cloud (j + 1, i)))
                  || (i + 1 < cloud.height && pcl::isNanPoint (cloud (j, i + 1)))
                  || (j + 1 < cloud.width && i + 1 < cloud.height && pcl::isNanPoint (cloud (j + 1, i + 1))))
              {
                added = true;
              }

              if ( (j > 0 && pcl::isNanPoint (cloud (j - 1, i))) || (i > 0 && pcl::isNanPoint (cloud (j, i - 1)))
                  || (j > 0 && i > 0 && pcl::isNanPoint (cloud (j - 1, i - 1))))
              {
                added = true;
              }

              if (added && indices[i][j] != 1 && !pcl::isNanPoint (cur))
              {
                indices[i][j] = 1;
                outcloud->push_back (cur);
              }
            }
          }
        }

        void saveLinesPCD (const LineSegment3DVector& lines, std::string filename)
        {

          pcl::PointCloud<PointOut> cloud;
          for (size_t i = 0; i < lines.size (); i++)
          {
            const LineSegment3D<PointOut>& line = lines[i];
            typename ::pcl::PointCloud<PointOut>::Ptr lineCloud = line.getCloud ();
            cloud.insert (cloud.end (), lineCloud->begin (), lineCloud->end ());
          }
          pcl::io::saveMoPcd<PointOut> (filename.c_str(), cloud);
        }

        LineSegment3DVector colorLines(LineSegment3DVector lines, float color) {
          for (size_t i = 0; i < lines.size(); i++) {
            LineSegment3D<PointOut>& line = lines[i];
            typename pcl::PointCloud<PointOut>::Ptr cloud = line.getCloud();
            pcl::colorCloud(*cloud, color);
            line.setCloud(cloud);
          }
          return lines;
        }

        void saveColoredLinesPCD (const LineSegment3DVector& lines, std::string filename)
        {

          pcl::PointCloud<PointOut> cloud;
          for (size_t i = 0; i < lines.size (); i++)
          {
            const LineSegment3D<PointOut>& line = lines[i];
            typename ::pcl::PointCloud<PointOut>::Ptr lineCloud = line.getCloud ();
            pcl::colorCloud (*lineCloud, pcl::color::getRandomColor ());
            cloud.insert (cloud.end (), lineCloud->begin (), lineCloud->end ());
          }
          if (cloud.size () > 0)
          {
            pcl::io::savePCDFileBinary (filename, cloud);
          } else {
            std::cerr << "cannot save lines: " << filename.c_str() << std::endl;
          }
         }

        void logLines (const LineSegment3DVector& lines)
        {
          printf ("numlines=%d\n", (int) lines.size ());
          for (size_t i = 0; i < lines.size (); i++)
          {
            const LineSegment3D<PointOut>& line = lines[i];
            printf ("line%d: length=%f, c=(%f,%f,%f), n=(%f,%f,%f), length=%f, p1=(%f,%f,%f), p2=(%f,%f,%f)\n",
                line.getId (), line.getLength (), line.getCenter ()[0], line.getCenter ()[1], line.getCenter ()[2],
                line.getNormal ()[0], line.getNormal ()[1], line.getNormal ()[2], line.getLength (),
                line.getLineModel  ()[0].x, line.getLineModel  ()[0].y, line.getLineModel  ()[0].z, line.getLineModel  ()[1].x,
                line.getLineModel  ()[1].y, line.getLineModel  ()[1].z);

          }
        }

        void findVerticalLines (const LineSegment3DVector& inLines)
        {
          float angleThreshold = 20;
          verticalLines.clear ();
          for (size_t i = 0; i < inLines.size (); i++)
          {
            const LineSegment3D<PointOut>& line = inLines[i];
            Eigen::Vector3f zero (0, 0, 1);
            if (areParallel (zero, line.getNormal (), angleThreshold))
            {
              verticalLines.push_back (line);
            }
          }
        }

      public:
        using pcl::PCLBase<PointIn>::input_;

        /** \brief Provide a pointer to the input dataset
         * \param cloud the const boost shared pointer to a PointCloud message
         */
        virtual inline void setInputCloud (const PointCloudConstPtr &cloud)
        {
          input_ = cloud;
        }

        LineSegment3DVector computeFromNormalImg ()
        {
          LineSegment3DVector lines;
          cv::Mat mat, rgbMat;
          std::vector<std::vector<int> > indices;
          pcl::opencv::cloud2Normals (*input_, mat);
          pcl::opencv::cloudToRGB (*input_, rgbMat);
          conc.addImage (rgbMat);
          extractEdgesContours (rgbMat, mat, lines, 40, 60, 120);
          //logLines(lines);
          //logLines (lines);
          return lines;
        }

        LineSegment3DVector computeFromDepthJumps ()
        {
          LineSegment3DVector lines;
          typename pcl::PointCloud<PointOut>::Ptr outCloud (new pcl::PointCloud<PointOut>);
          std::vector<std::vector<int> > indices;
          typename pcl::PointCloud<PointIn>::Ptr cloud (new pcl::PointCloud<PointIn>);
          //edges from depth jumps
          typename pcl::PointCloud<PointIn>::Ptr inputCloud (new pcl::PointCloud<PointIn>);
          pcl::copyPointCloud(*input_, *inputCloud);
          pcl::colorCloud(*inputCloud, generateColor(200,200,200));
//          pcl::io::saveMoPcd ("input_depthjumps.pcd", *inputCloud);
          cloud2DepthJumpMatrix (*input_, 2, 0.1, indices, cloud);

//          if (cloud->size () > 0)
//          {
////            float color = pcl::color::getColorFloatFromRGB (0, 0, 0);
////            pcl::colorCloud (*cloud, color);
//            pcl::io::saveMoPcd("depth_jumps.pcd", *cloud);
//          }

          typename pcl::PointCloud<PointOut>::Ptr cloud2 (new pcl::PointCloud<PointOut>);
          pcl::copyPointCloud (*cloud, *cloud2);
          extractEdgesfromCloud (cloud2, lines);
          lines = colorLines(lines, generateColor(50,50,50));
//          saveLinesPCD(lines, "depth_jumps_lines.pcd");
          std::sort (lines.begin (), lines.end ());
//          saveLinesPCD (lines, "initial_jumplines.pcd");
          //logLines (lines);
          //for find the railing
          findVerticalLines (lines);
          //saveLinesPCD(lines, "cloud1.pcd");
          lines = removeNonHorizontalLines<PointOut> (lines, 45);
//          saveLinesPCD(lines, "depth_jumps_filtered.pcd");
          //logLines (lines);
          LineSegment3DHistogram<PointOut> hist;
          std::vector<LineSegment3D<PointOut>, Eigen::aligned_allocator<LineSegment3D<PointOut> > > lines2;
          lines2.insert (lines2.end (), lines.begin (), lines.end ());
          hist.addLines (lines2);
          hist.createHistogram ();
          //hist.logHistogramStructure();
          //hist.logContents();
          hist.mergeLines ();
          //hist.logContents();
          lines2 = hist.getLines ();
          lines.clear ();
          lines.insert (lines.end (), lines2.begin (), lines2.end ());
//          saveLinesPCD(lines, "merged_lines.pcd");
//          saveColoredLinesPCD(lines, "merged_lines.pcd");
          //removeFarLines(lines, hist.getDominantNormal(), 20);
          //logLines (lines);
          return lines;
        }

        LineSegment3DVector getVerticalLines ()
        {
          return verticalLines;
        }
    };
  }
}
#endif /* EDGESDETECTOR_H_ */
