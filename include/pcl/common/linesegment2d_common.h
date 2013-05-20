/*
 * linesegment2d_utils.h
 *
 *  Created on: Jul 23, 2012
 *      Author: elmasry
 */

#ifndef LINESEGMENT2D_UTILS_H_
#define LINESEGMENT2D_UTILS_H_

#include <algorithm>

#include "opencv2/imgproc/imgproc.hpp"

#define SMALL_NUM  0.00000001 // anything that avoids division overflow
namespace pcl
{
  namespace common
  {
    inline bool
    IsOnSegment (double xi, double yi, double xj, double yj, double xk, double yk)
    {
      return (xi <= xk || xj <= xk) && (xk <= xi || xk <= xj) && (yi <= yk || yj <= yk) && (yk <= yi || yk <= yj);
    }

    inline char
    ComputeDirection (double xi, double yi, double xj, double yj, double xk, double yk)
    {
      double a = (xk - xi) * (yj - yi);
      double b = (xj - xi) * (yk - yi);
      return a < b ? -1 : a > b ? 1 : 0;
    }

    /** Do line segments (x1, y1)--(x2, y2) and (x3, y3)--(x4, y4) intersect? */
    inline bool
    isLineSegmentsIntersectInfinite (int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
    {
      char d1 = ComputeDirection (x3, y3, x4, y4, x1, y1);
      char d2 = ComputeDirection (x3, y3, x4, y4, x2, y2);
      char d3 = ComputeDirection (x1, y1, x2, y2, x3, y3);
      char d4 = ComputeDirection (x1, y1, x2, y2, x4, y4);
      return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) || (d1 == 0
          && IsOnSegment (x3, y3, x4, y4, x1, y1)) || (d2 == 0 && IsOnSegment (x3, y3, x4, y4, x2, y2)) || (d3 == 0
          && IsOnSegment (x1, y1, x2, y2, x3, y3)) || (d4 == 0 && IsOnSegment (x1, y1, x2, y2, x4, y4));
    }

    inline bool
    isLineSegmentsIntersectFinite (int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
    {
      int a1 = y2 - y1;
      int b1 = x1 - x2;
      //int c1 = x2 * y1 - x1 * y2; //{ a1*x + b1*y + c1 = 0 is line 1 }

      int a2 = y4 - y3;
      int b2 = x3 - x4;
      //int c2 = x4 * y3 - x3 * y4; //{ a2*x + b2*y + c2 = 0 is line 2 }

      int denom = a1 * b2 - a2 * b1;
      if (denom == 0)
      {
        return false;
      }
      else
      {
        //calculate intersecting point
        return true;
        //x:=(b1*c2 - b2*c1)/denom;
        //y:=(a2*c1 - a1*c2)/denom;
        //code:=0
      }
    }

    template<typename T>
      inline float
      perp (T u, T v)
      {
        return ((u[0] * v[1]) - (u[1] * v[0]));
      }

    /**
     * \param P has length of 2
     * \param S has length 4
     */
    template<typename T1, typename T2>
      int
      inSegment (T1 P, T2 S)
      {
        //if (S.P0.x != S.P1.x)
        if (S[0] != S[2])
        { // S is not vertical
          if (S[0] <= P[0] && P[0] <= S[2])
            return 1;
          if (S[0] >= P[0] && P[0] >= S[2])
            return 1;
        }
        else
        { // S is vertical, so test y coordinate
          if (S[1] <= P[1] && P[1] <= S[3])
            return 1;
          if (S[1] >= P[1] && P[1] >= S[3])
            return 1;
        }
        return 0;
      }

    /**
     * \return 0:parallel, 1:intersects at one point, 2: overlaps  in segment from I0 to I1
     */
    template<typename T>
      int
      intersect2D_Segments (T s1p0, T s1p1, T s2p0, T s2p1, T I0, T I1)
      {
        cv::Point (0, 0);
        Eigen::Vector2f u (s1p1[0] - s1p0[0], s1p1[1] - s1p0[1]);
        Eigen::Vector2f v (s2p1[0] - s2p0[0], s2p1[1] - s2p0[1]);
        Eigen::Vector2f w (s1p0[0] - s2p0[0], s1p0[1] - s2p0[1]);

        float D = perp (u, v);

        // test if they are parallel (includes either being a point)
        if (fabs (D) < SMALL_NUM)
        { // S1 and S2 are parallel
          if (perp (u, w) != 0 || perp (v, w) != 0)
          {
            return 0; // they are NOT collinear
          }
          // they are collinear or degenerate
          // check if they are degenerate points
          float du = u.dot (u);
          float dv = v.dot (v);
          if (du == 0 && dv == 0)
          { // both segments are points
            //if (S1.P0 != S2.P0) // they are distinct points
            if (s1p0[0] != s2p0[0] || s1p0[1] != s2p0[1]) // they are distinct points
              return 0;
            I0 = s1p0; // they are the same point
            return 1;
          }
          if (du == 0)
          { // S1 is a single point
            if (inSegment (s1p0, cv::Vec4i (s2p0[0], s2p0[1], s2p1[0], s2p1[1])) == 0) // but is not in S2
              return 0;
            I0 = s1p0;
            return 1;
          }
          if (dv == 0)
          { // S2 a single point
            if (inSegment (s2p0, cv::Vec4i (s1p0[0], s1p0[1], s1p1[0], s1p1[1])) == 0) // but is not in S1
              return 0;
            I0 = s2p0;
            return 1;
          }
          // they are collinear segments - get overlap (or not)
          float t0, t1; // endpoints of S1 in eqn for S2
          Eigen::Vector2f w2 (s1p1[0] - s2p0[0], s1p1[1] - s2p0[1]);
          if (v[0] != 0)
          {
            t0 = w[0] / v[0];
            t1 = w2[0] / v[0];
          }
          else
          {
            t0 = w[1] / v[1];
            t1 = w2[1] / v[1];
          }
          if (t0 > t1)
          { // must have t0 smaller than t1
            float t = t0;
            t0 = t1;
            t1 = t; // swap if not
          }
          if (t0 > 1 || t1 < 0)
          {
            return 0; // NO overlap
          }
          t0 = t0 < 0 ? 0 : t0; // clip to min 0
          t1 = t1 > 1 ? 1 : t1; // clip to max 1
          if (t0 == t1)
          { // intersect is a point
            I0 = s2p0 + t0 * v;
            return 1;
          }

          // they overlap in a valid subsegment
          I0 = T (s2p0 + t0 * v);//S2.P0 + t0 * v;
          I1 = T (s2p0 + t1 * v); //S2.P0 + t1 * v;
          return 2;
        }

        // the segments are skew and may intersect in a point
        // get the intersect parameter for S1
        float sI = perp (v, w) / D;
        if (sI < 0 || sI > 1) // no intersect with S1
          return 0;

        // get the intersect parameter for S2
        float tI = perp (u, w) / D;
        if (tI < 0 || tI > 1) // no intersect with S2
          return 0;

        I0 = T (s1p0 + sI * u);//S1.P0 + sI * u; // compute S1 intersect point
        return 1;
      }

    LineSegment2D
    merge2Lines (LineSegment2D& line1, LineSegment2D& line2, float dominantAngle)
    {
      float avgTheta = (line1.getThetaDegree () + line2.getThetaDegree ()) / 2;
      cv::Vec4i model1 = line1.getLineModel (), model2 = line2.getLineModel ();
      cv::Vec4i model;

      //Horizontal line
      float ang = fabs (line2.getTheta () - dominantAngle) < fabs (line1.getTheta () - dominantAngle)
          ? line2.getTheta () : line1.getTheta ();
      if (avgTheta > 45 && avgTheta < 135)
      {

        model[0] = std::min (std::min (model1[0], model2[0]), std::min (model1[2], model2[2]));
        model[2] = std::max (std::max (model1[0], model2[0]), std::max (model1[2], model2[2]));
        if (fabs (pcl::radianToAngle (dominantAngle) - line1.getThetaDegree ()) < 20
            && fabs (pcl::radianToAngle (dominantAngle) - line2.getThetaDegree ()) < 20)
        {//make the line aligned to the dominant orientation
          model[1] = LineSegment2D::findY (model[0], ang, (line1.getR () + line2.getR ()) / 2);
          model[3] = LineSegment2D::findY (model[2], ang, (line1.getR () + line2.getR ()) / 2);
        }
        else
        {
          model[1] = (model1[1] + model2[1]) / 2;
          model[3] = (model1[3] + model2[3]) / 2;
        }

      }
      else
      {
        model[1] = std::min (std::min (model1[1], model1[3]), std::min (model2[1], model2[3]));
        model[3] = std::max (std::max (model1[1], model1[3]), std::max (model2[1], model2[3]));
        if (fabs (pcl::radianToAngle (dominantAngle) - line1.getThetaDegree ()) < 20
            && fabs (pcl::radianToAngle (dominantAngle) - line2.getThetaDegree ()) < 20)
        {//make the line aligned to the dominant orientation
          model[0] = LineSegment2D::findX (model[1], ang, (line1.getR () + line2.getR ()) / 2);
          model[2] = LineSegment2D::findX (model[3], ang, (line1.getR () + line2.getR ()) / 2);
        }
        else
        {
          model[0] = (model1[0] + model2[0]) / 2;
          model[2] = (model1[2] + model2[2]) / 2;
        }
      }

      LineSegment2D outLine(line1.getId());
      outLine.setLineModel (model);
      return outLine;
    }
    void
    mergeNeighboringLines (std::vector<pcl::LineSegment2D>& lines, float dominantAngle, float angleThreshold = 20,
                           float maxDistanceThreshold = 6)
    {
      std::vector<pcl::LineSegment2D> tmpLines;

      std::sort (lines.begin (), lines.end ());
      for (size_t i = 0; i < lines.size (); i++)
      {
        LineSegment2D& line = lines[i];
        //printf("handlingline:%d, ", line.getId());
        //printf("line:%d center: (%d,%d)\n", line.getId(), line.getCentroid()[0], line.getCentroid()[1] );
        if (i + 1 < lines.size ())
        {
          LineSegment2D& line2 = lines[i + 1];
          if (pcl::calcAngle2DDegree (line.getNormal (), line2.getNormal ()) < angleThreshold
              && fabs (line.getCentroid ()[1] - line2.getCentroid ()[1]) <= maxDistanceThreshold)
          {
            //printf(", mergingwith: %d, ", line2.getId());
            LineSegment2D outLine = merge2Lines (line, line2, dominantAngle);
            tmpLines.push_back (outLine);
            //skip the next line
            i++;
          } else {
            tmpLines.push_back (line);
          }
        }
        else
        {
          tmpLines.push_back (line);
        }
      }
      lines.clear ();
      lines.assign (tmpLines.begin (), tmpLines.end ());
    }

    void
    logLines (const std::vector<LineSegment2D>& lines)
    {
      printf("numLines=%d\n", (int)lines.size());
      for (size_t i = 0; i < lines.size(); i++) {
        const LineSegment2D& line = lines[i];
        printf("line:%d center=(%d,%d), r=%f, theta=%f\n", line.getId(), line.getCentroid()[0], line.getCentroid()[1], line.getR(), line.getThetaDegree());
      }
    }
  }

}
#endif /* LINESEGMENT2D_UTILS_H_ */
