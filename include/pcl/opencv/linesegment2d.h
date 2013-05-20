/*
 * linesegment.h
 *
 *  Created on: May 31, 2012
 *      Author: elmasry
 */

#ifndef LINESEGMENT2D_H_
#define LINESEGMENT2D_H_

#include <cmath>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>
#include <vector>
#include <Eigen/Dense>

#include "pcl/common/motime.h"
#include "pcl/common/math.h"

namespace pcl
{

  class LineSegment2D
  {

    typedef Eigen::aligned_allocator<cv::Vec2i> Alloc;

    typedef std::vector<cv::Vec2i, Alloc> Line;
  protected:

    int id;
    Line points;
    cv::Vec4i lineModel;
    cv::Vec2i Centroid;
    Eigen::Vector2f normal;
    float rho, theta;

  public:
    /**This variable is related to the histogram, and is between 0-1,
     * close to zero means close to upper bin, to one to lower bin*/
    float rhoWeight;
    float thetaWeight;

    LineSegment2D ()
    {
    }
    LineSegment2D (int id)
    {
      this->id = id;
    }

    int
    getId () const
    {
      return id;
    }

  protected:
    void
    calculateNormal ()
    {
      normal[0] = (lineModel[3] - lineModel[1]);
      normal[1] = (lineModel[2] - lineModel[0]) * -1;
      normal.normalize ();

    }

    void
    calculateCentroid ()
    {

      Centroid[0] = (lineModel[0] + lineModel[2]) / 2;
      Centroid[1] = (lineModel[1] + lineModel[3]) / 2;
    }

    void
    constructLine ()
    {
      points.clear ();
      int horDist = lineModel[2] - lineModel[0];
      int vertDist = lineModel[3] - lineModel[1];

      float dist = sqrt (pow (horDist, 2) + pow (vertDist, 2));
      float xstep = horDist / dist;
      float ystep = vertDist / dist;
      float xbegin = lineModel[0];
      float ybegin = lineModel[1];
      if (abs (horDist > abs (vertDist)))
      {
        for (int i = 0; i < horDist; i++)
        {
          cv::Vec2i p ((int)floor (xbegin), (int)floor (ybegin));
          points.push_back (p);
          xbegin += xstep;
          ybegin += ystep;
        }
      }
      else
      {
        for (int i = 0; i < vertDist; i++)
        {
          cv::Vec2i p ((int)floor (xbegin), (int)floor (ybegin));
          points.push_back (p);
          xbegin += xstep;
          ybegin += ystep;
        }
      }
    }

    /**
     * calculates length of rho and theta as defined in hough line transform
     * http://en.wikipedia.org/wiki/Hough_transform
     *
     */
    /*void
     calculateThetaR2 ()
     {
     float A = sqrt (pow (lineModel[2], 2) + pow (lineModel[3], 2));
     float B = sqrt (pow (lineModel[0], 2) + pow (lineModel[1], 2));
     float C = sqrt (pow (lineModel[2] - lineModel[0], 2) + pow (lineModel[3] - lineModel[1], 2));
     float sinb = B / C;
     //calculate rho
     rho = A * sinb;
     float angB = asin (sinb);
     theta = 90 - angB;
     }*/

    /**
     * calculates length of rho and theta as defined in hough line transform
     * http://en.wikipedia.org/wiki/Hough_transform
     *
     */
    /*void
     calculateThetaR2 ()
     {
     float a = sqrt (pow (lineModel[2], 2) + pow (lineModel[3], 2));
     float b = sqrt (pow (lineModel[0], 2) + pow (lineModel[1], 2));
     float c = sqrt (pow (lineModel[2] - lineModel[0], 2) + pow (lineModel[3] - lineModel[1], 2));
     float cosA = (pow (b, 2) + pow (c, 2) - pow (a, 2)) / (2 * b * c);
     float angA = acos (cosA);
     theta = 90 - angA;
     float r1 = lineModel[0] * cos (theta) + lineModel[1] * sin (theta);
     //float r2 = lineModel[2] * cos (theta) + lineModel[3] * sin (theta);
     rho = r1;
     }*/

    void
    calculateThetaR ()
    {
      Eigen::Vector2f orig (-1, 0);
      //float thetaDeg = getSignedAngle(normal, orig);
      float thetaDeg = pcl::getAngle (orig, normal);
      //float thetaDeg = pcl::getSignedAngle (orig, normal);

      if (thetaDeg < 0)
      {
        thetaDeg = (float)((int)thetaDeg % 180);
        thetaDeg += 180;
      }
      else if (thetaDeg >= 180)
      {
        thetaDeg = (float)((int)thetaDeg % 180);
      }
      theta = pcl::degreeToRadian (thetaDeg);
      rho = Centroid[0] * cos (theta) + Centroid[1] * sin (theta);
    }

    void
    calculateLineModel ()
    {
      double a = cos (theta), b = sin (theta);
      double x0 = a * rho, y0 = b * rho;

      lineModel[0] = cvRound (x0 + 1000 * (-b));
      lineModel[1] = cvRound (y0 + 1000 * (a));
      lineModel[2] = cvRound (x0 - 1000 * (-b));
      lineModel[3] = cvRound (y0 - 1000 * (a));

      //lineModel[0] = cvRound (x0  * (-b));
      //lineModel[1] = cvRound (y0  * (a));
      //lineModel[2] = cvRound (x0  * (-b));
      //lineModel[3] = cvRound (y0  * (a));
    }

  public:

    LineSegment2D (const LineSegment2D& line)
    {
      points = line.points;
      lineModel = line.lineModel;
      Centroid = line.Centroid;
      normal = line.normal;
      id = line.id;
      rho = line.rho;
      theta = line.theta;
      rhoWeight = line.rhoWeight;
      thetaWeight = line.thetaWeight;
      id = line.id;
    }

    LineSegment2D&
    operator= (const LineSegment2D& line)
    {
      points = line.points;
      lineModel = line.lineModel;
      Centroid = line.Centroid;
      normal = line.normal;
      id = line.id;
      rho = line.rho;
      theta = line.theta;
      rhoWeight = line.rhoWeight;
      thetaWeight = line.thetaWeight;
      id = line.id;
      return (*this);
    }

    friend std::ostream&
    operator << (std::ostream& os, const LineSegment2D& l)
    {
      os << "theta=" << l.theta << ", rho=" << l.rho << "center=(" << l.Centroid[0] << "," << l.Centroid[1]
          << "), model=(";
      os << l.lineModel[0] << "," << l.lineModel[1] << "," << l.lineModel[2] << "," << l.lineModel[3] << ")"
          << std::endl;
      return (os);
    }

    bool
    operator < (const LineSegment2D& b) const
    {
      return this->Centroid[1] < b.Centroid[1];
    }

    void
    setLineModel (cv::Vec4i model)
    {
      lineModel = model;
      constructLine ();
      //calculate centroid
      calculateCentroid ();
      calculateNormal ();
      calculateThetaR ();
    }

    /**
     * width and height of the image
     */
    void
    setLineModel (float rho, float theta)
    {
      this->rho = rho;
      this->theta = theta;
      calculateLineModel ();
      constructLine ();
      calculateCentroid ();
      calculateNormal ();
    }

    void
    setLineModel (float rho, float theta, cv::Vec4i model)
    {
      this->rho = rho;
      this->theta = theta;
      this->lineModel = model;
      constructLine ();
      calculateCentroid ();
      calculateNormal ();
    }

    inline Line
    getPoints ()
    {
      return points;
    }

    inline cv::Vec4i
    getLineModel ()
    {
      return lineModel;
    }

    inline const Eigen::Vector2f
    getNormal () const
    {
      return normal;
    }

    inline cv::Vec2i
    getCentroid () const
    {
      return Centroid;
    }

    inline float
    getR () const
    {
      return rho;
    }

    inline float
    getTheta () const
    {
      return theta;
    }

    inline float
    getThetaDegree () const
    {
      return pcl::radianToAngle (theta);
    }

    inline float
    getLength ()
    {
      return distance2D (cv::Vec2i (lineModel[0], lineModel[1]), cv::Vec2i (lineModel[2], lineModel[3]));
    }

    inline bool
    isHorizontal ()
    {
      float thetaDegree = getThetaDegree ();
      if (thetaDegree > 70 && thetaDegree < 110)
      {
        return true;
      }
      return false;
    }

    inline bool
    isVertical ()
    {
      float thetaDegree = getThetaDegree ();
      if (thetaDegree < 20 && thetaDegree > 160)
      {
        return true;
      }
      return false;
    }
    /**
     * calculates x coordinate from the given values. theta is in radian
     */
    template<typename T>
      static inline int
      findX (int y, T theta, T rho)
      {
        return (rho - y * sin (theta)) / cos (theta);
      }

    /**
     * calculates y coordinate from the given values. theta is in radian
     */
    template<typename T>
      static inline int
      findY (int x, T theta, T rho)
      {
        return (rho - x * cos (theta)) / sin (theta);
      }
  };

}

#endif /* LINESEGMENT2D_H_ */
