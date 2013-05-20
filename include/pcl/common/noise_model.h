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

#ifndef PCL_COMMON_NOISE_MODEL_H_
#define PCL_COMMON_NOISE_MODEL_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace pcl
{

  /** \brief Check whether or not the given is valid (has valid values for x, y, and z)
   *  \param point to be checked */
  template <typename PointT> inline bool isValid(const PointT& point)
  {
    if (pcl_isfinite(point.x) && pcl_isfinite(point.y) && pcl_isfinite(point.z) )
      return true;
    return false;
  }



//  template <typename PointT>
//  class NoiseModel
//  {
//    public:
//      typedef pcl::PointCloud<PointT> PointCloud;
//      typedef typename PointCloud::Ptr PointCloudPtr;
//      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
//
//      NoiseModel(){}
//  };
//
//  template <typename PointT, const float& a, const float& b, const float& c>
//  class QuadraticIsotropicNoiseModel : public NoiseModel<PointT>
//  {
//    QuadraticIsotropicNoiseModel(){}
//  };
//
//  template <typename PointT> typedef QuadraticIsotropicNoiseModel<PointT, 0.1f, 0.1f, 0.1f> NoiseModelHolz;


  /**
   * \brief Abstract implementation of a camera noise model
   * \author Dirk Holz
   * \ingroup common
   */
  template <typename PointT>
  class NoiseModel
  {
      typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
      typedef boost::shared_ptr <const std::vector<int> > IndicesConstPtr;

    public:
      typedef boost::shared_ptr< NoiseModel<PointT> > Ptr;
      typedef boost::shared_ptr< const NoiseModel<PointT> > ConstPtr;

      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      NoiseModel(){};

      /** \brief Provide a pointer to the input dataset
        * \param cloud the const boost shared pointer to a PointCloud message
        * \param indices of points to use (if 0, the complete is complete cloud is processed)
        */
      inline void
      setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices = IndicesConstPtr ()) {}


      /** \brief Get local standard deviation depending on range measurement (other information stored in the input cloud)
        * \param index of the point to lookup the standard deviation
        *
        */
      virtual float
      getStdDev(const int& index) { return 0.0f; }

      /**
       * Get weight depending on standard deviation (should be something )
       */
      virtual inline float
      getWeight(const int& index)
      {
        const float sigma = getStdDev(index);
        if (sigma < 1e-9)
          return 0.0f;
        return (1/(sigma*sigma));
      }
  };

//  template <const float& a, const float& b, const float& c>
  template <typename PointT>
  class QuadraticIsotropicNoise : public NoiseModel<PointT>
  {
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      typedef boost::shared_ptr< QuadraticIsotropicNoise<PointT> > Ptr;
      typedef boost::shared_ptr< const QuadraticIsotropicNoise<PointT> > ConstPtr;

      QuadraticIsotropicNoise() : a_(0.0f), b_(0.0f), c_(0.0f) {}
      QuadraticIsotropicNoise(const float& a, const float& b, const float& c) : a_(a), b_(b), c_(c) {}

      inline void
      setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices = IndicesConstPtr ())
      {
        int nr_points = (int)cloud->points.size();
        standard_deviations_.resize(nr_points);
        weights_.resize(nr_points);

        const float INVALID_STANDARD_DEVIATION = 0.0f; // TODO: reasonable?!
        const float INVALID_WEIGHT = 1.0f;

        if (indices) // here we assume that all indices belong to valid points
        {
          for (int i = 0; i < nr_points; ++i)
          {
            standard_deviations_[i] = INVALID_STANDARD_DEVIATION;
            weights_[i] = INVALID_WEIGHT;
          }
          int nr_indices = indices->size();
          for (int i = 0; i < nr_indices; ++i)
          {
            const int& index = (*indices)[i];
            const float x = cloud->points[index].getVector3fMap().norm();
            standard_deviations_[index] = a_*x*x + b_*x + c_*x;
            weights_[index] = 1 / (standard_deviations_[index] * standard_deviations_[index]);
          }
        }
        else
        {
          for (int i = 0; i < nr_points; ++i)
          {
            const PointT& point = cloud->points[i];
            if (pcl::isValid(point))
            {
              const float x = point.getVector3fMap().norm();
              standard_deviations_[i] = a_*x*x + b_*x + c_*x;
              weights_[i] = 1 / (standard_deviations_[i] * standard_deviations_[i]);
            }
            else
            {
              standard_deviations_[i] = INVALID_STANDARD_DEVIATION;
              weights_[i] = INVALID_WEIGHT;
            }
          }
        }
      }

      virtual float
      getStdDev(const int& index) { return standard_deviations_[index]; }

      virtual inline float
      getWeight(const int& index) { return weights_[index]; }

      inline void setModelHolz()
      {
        a_ = +0.00263146f;
        b_ = -0.00517505f;
        c_ = +0.00752267f;
      }

      inline void setModelSegComp()
      {
        a_ = +0.005f;
        b_ = 0.0f;
        c_ = +0.01f;
      }

    inline void setModelBremen()
      {
        a_ = -0.000243897f;
        b_ = +0.00486625f;
        c_ = -0.0007691149f;
      }

      inline void setModelAnderson()
      {
        a_ = 0.0018f;
        b_ = 0.0f;
        c_ = 0.0f;
      }

    protected:
      std::vector<float> standard_deviations_;
      std::vector<float> weights_;

      /** Coefficients */
      float a_, b_, c_;

  };




//  /** Noise model by Holz et al.: 0.00263*x^2 - 0.00518*x + 0.00752 */
//  typedef QuadraticIsotropicNoise<0.00263146f, -0.00517505f, 0.00752267f> NoiseModelHolzKinect;
//
//  /** Noise model (for the SwissRanger SR 3000/4000(?) camera by Pathak, Poppinga, Birk et al. */
//  typedef QuadraticIsotropicNoise<-0.000243897f, 0.00486625f, -0.0007691149f> NoiseModelBremenSR;
//
//  /** Noise model from Anderson, Herman, and Kelly for SwissRanger cameras */
//  typedef QuadraticIsotropicNoise<0.0018f, 0.0f, 0.0f> NoiseModelAndersonSR;

}

#endif /* PCL_COMMON_NOISE_MODEL_H_ */
