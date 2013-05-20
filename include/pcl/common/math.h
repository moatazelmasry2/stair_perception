/*
 * math.h
 *
 *  Created on: Jun 3, 2012
 *      Author: elmasry
 */

#ifndef MATH_H_
#define MATH_H_

#include <math.h>
#include <Eigen/Dense>

namespace pcl
{

  inline float
  getAngle (Eigen::Vector3f vec1, Eigen::Vector3f vec2)
  {
    return acos ((vec1.dot (vec2)) / (vec1.norm () * vec2.norm ())) * 180 / M_PI;
  }

  inline float
  getAngle (Eigen::Vector2f vec1, Eigen::Vector2f vec2)
  {
    return acos ((vec1.dot (vec2)) / (vec1.norm () * vec2.norm ())) * 180 / M_PI;
  }

  /**
   * returns angle in degrees
   */
  inline double
  getAngle (Eigen::Vector3d vec1, Eigen::Vector3d vec2)
  {
    return acos ((vec1.dot (vec2)) / (vec1.norm () * vec2.norm ())) * 180 / M_PI;
  }

  /**
   * returns the ange with +- sign in degrees
   */
  inline float
  getSignedAngle (Eigen::Vector2f vec1, Eigen::Vector2f vec2)
  {
    return atan2 (vec2[1], vec2[0]) - atan2 (vec1[1], vec1[0]) * 180 / M_PI;

  }

  inline float
  degreeToRadian (float angle)
  {
    return angle * M_PI / 180;
  }

  inline float
  radianToAngle (float rad)
  {
    return rad * 180 / M_PI;
  }

  /**
   * Translates a number in a number range to a number in another range
   */
  template<typename T>
    T
    translateNumberRange (T OldValue, T OldMin, T OldMax, T NewMin, T NewMax)
    {
      return (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin;
    }

  template<typename T>
    inline float
    distance2D (T vec1, T vec2)
    {
      return sqrt (pow (vec2[0] - vec1[0], 2) + pow (vec2[1] - vec1[1], 2));
    }

  inline float
  distance3D (Eigen::Vector3f vec1, Eigen::Vector3f vec2)
  {
    return sqrt (pow (vec2[0] - vec1[0], 2) + pow (vec2[1] - vec1[1], 2) + pow (vec2[2] - vec1[2], 2));
  }

  /**
   * returns angle between vectors in radian
   */
  inline double
  calcAngle3D (const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
  {
    // Compute the actual angle
    double rad = v1.dot (v2) / sqrt (v1.squaredNorm () * v2.squaredNorm ());
    if (rad < -1.0) rad = -1.0;
    if (rad > 1.0) rad = 1.0;
    return (acos (rad));
  }

  /**
   * returns angle between vectors in radian
   */
  inline double
  calcAngle3DDegree (const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
  {
    return calcAngle3D (v1, v2) * 180 / M_PI;
  }

  /**
   * returns angle between vectors in radian
   */
  inline double
  calcAngle2D (const Eigen::Vector2f &v1, const Eigen::Vector2f &v2)
  {
    // Compute the actual angle
    double rad = v1.dot (v2) / sqrt (v1.squaredNorm () * v2.squaredNorm ());
    if (rad < -1.0) rad = -1.0;
    if (rad > 1.0) rad = 1.0;
    return (acos (rad));
  }

  /**
   * returns angle between vectors in radian
   */
  inline double
  calcAngle2DDegree (const Eigen::Vector2f &v1, const Eigen::Vector2f &v2)
  {
    return calcAngle2D (v1, v2) * 180 / M_PI;
  }

  inline bool
  areParallel (const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, float toleranceAngle)
  {
    float angle = calcAngle3DDegree (v1, v2);
    return angle <= toleranceAngle || (180 - toleranceAngle) < angle;
  }

  inline bool
  isHorizontal (Eigen::Vector3f vec1, Eigen::Vector3f vec2, float ANGLE_THRESHHOLD = 15)
  {
    float angle = getAngle (vec1, vec2);
    if (angle < ANGLE_THRESHHOLD || fabs (180 - angle) < ANGLE_THRESHHOLD) return true;
    return false;
  }

  inline bool
  isVertical (Eigen::Vector3f vec1, Eigen::Vector3f vec2, float ANGLE_THRESHHOLD = 15)
  {
    float angle = getAngle (vec1, vec2);
    if (fabs (90 - angle) < ANGLE_THRESHHOLD) return true;
    return false;
  }

}
#endif /* MATH_H_ */
