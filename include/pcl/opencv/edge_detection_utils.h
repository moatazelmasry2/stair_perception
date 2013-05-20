/*
 * edge_detection_utils.h
 *
 *  Created on: Jul 16, 2012
 *      Author: elmasry
 */

#ifndef EDGE_DETECTION_UTILS_H_
#define EDGE_DETECTION_UTILS_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "pcl/opencv/linesegment2d.h"

namespace pcl
{
  namespace opencv
  {
    void
    removeShortLines (std::vector<LineSegment2D>& lines, float minLength = 10)
    {
      std::vector<LineSegment2D> tmpLines;
      for (size_t i = 0; i < lines.size (); i++)
      {
        LineSegment2D& line = lines[i];
        if (line.getLength () >= minLength)
        {
          tmpLines.push_back (line);
        }
      }
      lines.clear ();
      lines.assign (tmpLines.begin (), tmpLines.end ());
    }

    /**
     * remove lines close to the borders. At the moment only removess horizontal lines close to upper/bottom borders
     * border, height of the image
     */
    void
    removeBorderLines (std::vector<LineSegment2D>& lines, int width, int height, float borderDist = 30)
    {
      std::vector<LineSegment2D> tmpLines;
      for (size_t i = 0; i < lines.size (); i++)
      {
        LineSegment2D& line = lines[i];
        if (line.getCentroid ()[1] > borderDist && (fabs (height - line.getCentroid ()[1])) > borderDist)
        {
          tmpLines.push_back (line);
        }
      }
      lines.clear ();
      lines.assign (tmpLines.begin (), tmpLines.end ());
    }

    void
    removeVerticalLines (std::vector<LineSegment2D>& lines)
    {
      std::vector<LineSegment2D> tmpLines;
      for (size_t i = 0; i < lines.size (); i++)
      {
        LineSegment2D& line = lines[i];
        if (line.isHorizontal ())
        {
          tmpLines.push_back (line);
        }
      }
      lines.clear ();
      lines.assign (tmpLines.begin (), tmpLines.end ());
    }
  }
}
#endif /* EDGE_DETECTION_UTILS_H_ */
