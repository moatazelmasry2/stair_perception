/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Dirk Holz, University of Bonn.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id:$
 *
 */

#ifndef PCL_COMMON_COLOR_H_
#define PCL_COMMON_COLOR_H_

#include "pcl/pcl_macros.h"

namespace pcl
{
  namespace color
  {

    /** \brief Get PCL (bit-packed) color float from R/G/B
     * \param r the R value (from 0 to 255)
     * \param g the G value (from 0 to 255)
     * \param b the B value (from 0 to 255)
     */
    PCL_EXPORTS inline float
    getColorFloatFromRGB(const unsigned char& r, const unsigned char& g, const unsigned char& b)
    {
      int rgb = ( ((int)(r) << 16 ) | ((int)(g) << 8) | (int)(b) );
      float color_float = *reinterpret_cast<float*>(&rgb);
      return (color_float);
    }

    /** \brief Get R/G/B color from PCL (bit-packed) color float
     * \param color PCL color float (e.g. from a PointCloud2)
     * \param r the R value (from 0 to 255)
     * \param g the G value (from 0 to 255)
     * \param b the B value (from 0 to 255)
     */
    PCL_EXPORTS inline void
    getRGBFromColorFloat(const float& color, unsigned char& r, unsigned char& g, unsigned char& b)
    {
      float rgb_data = color;
      int rgb = *reinterpret_cast<int*>(&rgb_data);
      r = ((rgb >> 16) & 0xff);
      g = ((rgb >> 8) & 0xff);
      b = (rgb & 0xff);
    }

    /** \brief Get R/G/B color from PCL (bit-packed) color float
     * \param color PCL color float (e.g. from a PointCloud2)
     * \param r the R value (from 0.0 to 1.0)
     * \param g the G value (from 0.0 to 1.0)
     * \param b the B value (from 0.0 to 1.0)
     */
    PCL_EXPORTS inline void
    getRGBFromColorFloat (const float& color, float& r, float& g, float& b)
    {
      float rgb_float = color;
      int rgb = *reinterpret_cast<int*> (&rgb_float);
      r = (float)((rgb >> 16) & 0xff) / 255.0f;
      g = (float)((rgb >> 8) & 0xff) / 255.0f;
      b = (float)(rgb & 0xff) / 255.0f;
    }

    /** \brief Get (good) random values for R/G/B.
     * \param r the resultant R color value (between 0.0 and 1.0)
     * \param g the resultant G color value (between 0.0 and 1.0)
     * \param b the resultant B color value (between 0.0 and 1.0)
     * \param min_sum (optional) minimum value for the colors (R+G+B)
     * \param max_sum (optional) maximum value for the colors (R+G+B)
     */
    PCL_EXPORTS inline void
    getRandomColor (float &r, float &g, float &b, float min_sum = 0.2, float max_sum = 2.8)
    {
      float sum;
      static unsigned stepRGBA = 100;
      do
      {
        sum = 0;
        r = (rand () % stepRGBA) / (float)stepRGBA;
        while ((g = (rand () % stepRGBA) / (float)stepRGBA) == r)
        {
        }
        while (((b = (rand () % stepRGBA) / (float)stepRGBA) == r) && (b == g))
        {
        }
        sum = r + g + b;
      } while (sum <= min_sum || sum >= max_sum);
    }

    /** \brief Get (good) random values for R/G/B.
     * \param r the resultant R color value (between 0 and 255)
     * \param g the resultant G color value (between 0 and 255)
     * \param b the resultant B color value (between 0 and 255)
     * \param min (optional) minimum value for the colors (R+G+B)
     * \param max (optional) maximum value for the colors (R+G+B)
     */
    PCL_EXPORTS inline void
    getRandomColor (unsigned char& r, unsigned char& g, unsigned char& b, float min = 0.2, float max = 2.8)
    {
      float temp_r, temp_g, temp_b;
      getRandomColor(temp_r, temp_g, temp_b, min, max);
      r = (unsigned char)(temp_r * 255);
      g = (unsigned char)(temp_g * 255);
      b = (unsigned char)(temp_b * 255);
    }

    /** \brief Get (good) random color.
     * \param min (optional) minimum value for the colors (R+G+B)
     * \param max (optional) maximum value for the colors (R+G+B)
     */
    PCL_EXPORTS inline float
    getRandomColor (float min = 0.2, float max = 2.8)
    {
      unsigned char r, g, b;
      getRandomColor (r, g, b, min, max);
      return getColorFloatFromRGB (r, g, b);
    }

    /** \brief Get intensity form R/G/B color
     * See Arthur Broadbent,
     * "A critical review of the development of the CIE1931 RGB color-matching functions",
     * in Color Research and Application 29(4):267-272, Wiley Press, doi 10.1002/col.20020
     * \param r the R value (from 0.0 to 1.0)
     * \param g the G value (from 0.0 to 1.0)
     * \param b the B value (from 0.0 to 1.0)
     */
    PCL_EXPORTS inline float
    getIntensity (const float& r, const float& g, const float& b)
    {
      const float luminance = 0.2126f * r + 0.7152f * g + 0.0722f * b;
      //      const float luminance = 0.299f*r + 0.587f*g + 0.114f*b;
      //      const float luminance = 0.3f*r + 0.59f*g + 0.11f*b;
      return luminance;
    }

    /** \brief Get intensity form R/G/B color
     * \param r the R value (from 0 and 255)
     * \param g the G value (from 0 and 255)
     * \param b the B value (from 0 and 255)
     */
    PCL_EXPORTS inline float
    getIntensity(const unsigned char& r, const unsigned char& g, const unsigned char& b)
    {
      return getIntensity((float)r/255.0f, (float)g/255.0f, (float)b/255.0f);
    }

    PCL_EXPORTS inline float
    getIntensity (const float& color)
    {
      float r, g, b;
      getRGBFromColorFloat (color, r, g, b);
      return getIntensity (r, g, b);
    }
  }
}

//#include "pcl/common/impl/color.hpp"

#endif /* PCL_COMMON_COLOR_H_ */
