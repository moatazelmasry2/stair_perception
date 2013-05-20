/*
 * edges_common.h
 *
 *  Created on: Jul 31, 2012
 *      Author: elmasry
 */

#ifndef EDGES_COMMON_H_
#define EDGES_COMMON_H_

#include <pcl/common/distances.h>

#include "pcl/common/math.h"
#include "pcl/common/color.h"
#include "pcl/utils/pointcloud_utils.h"
#include "pcl/types/linesegment3d.h"

namespace pcl
{

  template<typename PointT>
  typename pcl::PointCloud<PointT>::Ptr lines2Cloud (
      std::vector<pcl::LineSegment3D<PointT>, Eigen::aligned_allocator<PointT> > lines, bool color = false)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    float white = pcl::generateColor (254, 254, 254);
    for (size_t i = 0; i < lines.size (); i++)
    {
      LineSegment3D<PointT>& line = lines[i];
      if (color)
      {
        float col = ::pcl::color::getRandomColor ();
        pcl::colorCloud<PointT> (line.getCloud (), col);
      }
      else
      {
        pcl::colorCloud<PointT> (line.getCloud (), white);
      }
      concatePointClouds (*line.getCloud (), *cloud);
    }
    return cloud;
  }

  template<typename PointT>
  void extractEdgesfromCloud (typename pcl::PointCloud<PointT>::Ptr cloud,
      std::vector<pcl::LineSegment3D<PointT>, Eigen::aligned_allocator<PointT> >& lines,
      float distance_threshold = 0.02, float max_iterations = 300, int numLoops = 30)
  {
    typename pcl::PointCloud<PointT>::Ptr tmpCloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud (*cloud, *tmpCloud);

    int count = 0;
    //add lines as long as ransac find lines
    pcl::ExtractIndices<PointT> extract;
    extract.setNegative (true);

    //for (int i = 0; i < numLoops; i++)
    while (tmpCloud->size () > 50)
    {
      // Find the dominant plane
      typename pcl::PointCloud<PointT>::Ptr lineCloud (new pcl::PointCloud<PointT>);
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients (false);
      seg.setModelType (pcl::SACMODEL_LINE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (distance_threshold);
      seg.setMaxIterations (max_iterations);
      seg.setInputCloud (tmpCloud);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size () == 0 || coefficients->values.size () == 0)
      {
        break;
      }

      Eigen::VectorXf model_coefficients;
      model_coefficients.resize (6);
      for (int i = 0; i < 6; i++)
      {
        model_coefficients[i] = coefficients->values[i];
      }

      typename pcl::SampleConsensusModel<PointT>::Ptr model = seg.getModel ();
      model->projectPoints (inliers->indices, model_coefficients, *lineCloud, false);
      pcl::LineSegment3D<PointT> line (count++, 0.03);
      //        PointT p1, p2;
      //        p1.x = model_coefficients[0];
      //        p1.y = model_coefficients[1];
      //        p1.z = model_coefficients[2];
      //        p2.x = model_coefficients[3];
      //        p2.y = model_coefficients[4];
      //        p2.z = model_coefficients[5];

      line.setCloud (lineCloud);
      //line.setLineModel (recalculateLineModel<PointT> (lineCloud));

      if (line.getLength () > 0.02 && lineCloud->size () > 50)
      {
        //std::vector<PointT, Eigen::aligned_allocator<PointT> > model = line.getModel ();
        //typename pcl::PointCloud<PointT>::Ptr outLine = reconstructLineSegment (model[0], model[1]);
        //line.setCloud(outLine);
        lines.push_back (line);
      }

      extract.setInputCloud (tmpCloud);
      pcl::IndicesPtr indices (new std::vector<int>);
      indices->resize (inliers->indices.size ());
      for (size_t i = 0; i < inliers->indices.size (); i++)
      {
        indices->operator [] (i) = inliers->indices[i];
      }
      extract.setIndices (indices);
      extract.filter (*tmpCloud);
      //printf ("tmpcloud.size=%d\n", (int)tmpCloud->size ());
    }
    //printf ("edges_common:numlines=%d\n", (int)lines.size ());
  }

  template<typename PointT>
  typename pcl::PointCloud<PointT>::Ptr reconstructLineSegment (const PointT& start, const PointT& end, int numPoints)
  {

    float color = ::pcl::color::getRandomColor ();
    return reconstructLineSegment (start, end, numPoints, color);
  }

  template<typename PointT>
  typename pcl::PointCloud<PointT>::Ptr reconstructLineSegment (const PointT& start, const PointT& end, int numPoints,
      float color)
  {
//    printf("reconstructing line (%f,%f,%f), (%f,%f,%f)\n", start.x, start.y, start.z, end.x, end.y, end.z);
    typename pcl::PointCloud<PointT>::Ptr outCloud (new pcl::PointCloud<PointT>);

    float xstep = (end.x - start.x) / numPoints;
    float ystep = (end.y - start.y) / numPoints;
    float zstep = (end.z - start.z) / numPoints;
//    printf("xstep=%f,ystep=%f, zstep=%f\n", xstep, ystep, zstep);

    PointT p = start;
    p.rgb = color;

    outCloud->push_back (p);
    for (int i = 0; i < numPoints; i++)
    {
      p.x += xstep;
      p.y += ystep;
      p.z += zstep;
      p.rgb = color;
//      printf("(%f,%f,%f), ", p.x, p.y, p.z);
      outCloud->push_back (p);
    }
//    printf("\n");
    p = end;
    p.rgb = color;
    outCloud->push_back (p);
    return outCloud;
  }

  template<typename PointT>
  typename pcl::PointCloud<PointT>::Ptr reconstructLineSegment (const PointT& start, const PointT& end, float stepSize)
  {
    int numPoints = pcl::euclideanDistance (start, end) / stepSize;
    return reconstructLineSegment (start, end, numPoints);
  }

  template<typename PointT>
  typename pcl::PointCloud<PointT>::Ptr reconstructLineSegment (const PointT& start, const PointT& end, float stepSize,
      float color)
  {
    int numPoints = pcl::euclideanDistance (start, end) / stepSize;
    return reconstructLineSegment (start, end, numPoints, color);
  }

  /**
   * \param angleThreshold angle threshold in degrees
   */
  template<typename PointT>
  void removeFarLines (std::vector<pcl::LineSegment3D<PointT>, Eigen::aligned_allocator<PointT> >& lines,
      Eigen::Vector3f domOrientation, float angleThreshold)
  {
    std::vector<pcl::LineSegment3D<PointT>, Eigen::aligned_allocator<PointT> > tmpLines;
    for (size_t i = 0; i < lines.size (); i++)
    {
      LineSegment3D<PointT>& line = lines[i];
      if (calcAngle3DDegree (domOrientation, line.getNormal ()) <= angleThreshold)
      {
        tmpLines.push_back (line);
      }
    }
    lines.clear ();
    lines.assign (tmpLines.begin (), tmpLines.end ());
  }

  template<typename PointT>
  std::vector<pcl::LineSegment3D<PointT>, Eigen::aligned_allocator<PointT> > removeNonHorizontalLines (
      const std::vector<pcl::LineSegment3D<PointT>, Eigen::aligned_allocator<PointT> >& lines,
      float angleThreshold = 20)
  {
    std::vector<pcl::LineSegment3D<PointT>, Eigen::aligned_allocator<PointT> > tmpLines;
    Eigen::Vector3f zero (0, 1, 0);
    for (size_t i = 0; i < lines.size (); i++)
    {
      const LineSegment3D<PointT>& line = lines[i];
      if (areParallel (zero, line.getNormal (), angleThreshold))
      {
        tmpLines.push_back (line);
      }
    }
    //printf("\n");
    return tmpLines;
  }
}
#endif /* EDGES_COMMON_H_ */
