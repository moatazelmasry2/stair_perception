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

#ifndef PCL_COMMON_INCREMENTAL_PLANE_FITTING_H_
#define PCL_COMMON_INCREMENTAL_PLANE_FITTING_H_

#include <pcl/common/eigen.h>
#include <pcl/pcl_base.h>

namespace pcl
{

  /** \brief Compute plane parameters (normal and distance to origin) and return smallest eigen value */
  inline float computePlaneParameters(const Eigen::Vector3f& centroid, const Eigen::Matrix3f& covariance, Eigen::Vector4f& parameters)
  {
    Eigen::Vector3f eigen_values;
    Eigen::Matrix3f eigen_vectors;
    pcl::eigen33 (covariance, eigen_vectors, eigen_values);

    // plane normal
    parameters[0] = eigen_vectors (0, 0);
    parameters[1] = eigen_vectors (1, 0);
    parameters[2] = eigen_vectors (2, 0);
    parameters.head<3>().normalize();

    parameters[3] = -1 * parameters.head<3>().dot (centroid);

    // hack to always have positive distances!!! (otherwise some approximations may experience singularities)
    if (parameters[3] < 0)
      parameters *= -1;

    return eigen_values(0);
  }

  template <typename PointT>
  inline float computePlaneParameters(const pcl::PointCloud<PointT>& cloud, const std::vector<int>& indices, Eigen::Vector4f& parameters)
  {
    Eigen::Vector4f centroid;
    compute3DCentroid (cloud, indices, centroid);
    centroid[3] = 0;

    Eigen::Matrix3f covariance_matrix;
    computeCovarianceMatrix (cloud, indices, centroid, covariance_matrix);

    return computePlaneParameters(centroid.head<3>(), covariance_matrix, parameters);
  }

  template <typename PointT>
  inline float computePlaneMSE(const pcl::PointCloud<PointT>& cloud, const std::vector<int>& indices, Eigen::Vector4f& parameters)
  {
    float mean_square_error = 0.0f;
    int nr_points = (int)indices.size();
    for (int i = 0; i < nr_points; ++i)
    {
      const float distance = (parameters.head<3>().dot(cloud.points[indices[i]].getVector3fMap())) + parameters(3);
      mean_square_error += distance*distance;
    }
    mean_square_error /= (float)(nr_points);
    return mean_square_error;
  }

  class PlaneModel
  {
    public:

      PlaneModel() : normal (1,0,0), distance (0.0f) {}
      PlaneModel(const Eigen::Vector3f& arg_normal, const float& arg_distance) : normal (arg_normal), distance (arg_distance) {}
      PlaneModel(const Eigen::Vector3f& centroid, const Eigen::Matrix3f& covariance)
      {
        fromCentroidAndCovariance(centroid, covariance);
      }

      PlaneModel(const Eigen::Vector3f& normal, const Eigen::Vector3f& centroid)
      {
        fromNormalAndCentroid(normal, centroid);
      }

      inline float fromCentroidAndCovariance(const Eigen::Vector3f& centroid, const Eigen::Matrix3f& covariance)
      {
        Eigen::Vector4f parameters;
        float lambda = pcl::computePlaneParameters(centroid, covariance, parameters);
        normal = parameters.head<3>();
        distance = parameters(3);
        return lambda;
      }

      inline void fromNormalAndCentroid(const Eigen::Vector3f& normal_arg, const Eigen::Vector3f& centroid_arg)
      {
        normal = normal_arg;
        distance = -normal.dot(centroid_arg);
        if (distance < 0)
        {
          normal *= -1;
          distance *= -1;
        }
      }

      Eigen::Vector3f normal;
      float distance;
  };


  template <typename PointT> inline float
  distPointPlane(const PointT& point, const Eigen::Vector4f& model_coefficients)
  {
    return (float)(model_coefficients.head<3>().dot(point.getVector3fMap()) + model_coefficients(3));
  }

  template <typename PointT> inline float
  distPointPlane(const PointT& point, const pcl::PlaneModel model)
  {
    return (float)(model.normal.dot(point.getVector3fMap()) + model.distance);
  }

  template <typename PointT> inline void
  projectPoints(pcl::PointCloud<PointT>& cloud_in, const std::vector<int>& indices, const Eigen::Vector4f& model_coefficients)
  {
    Eigen::Vector4f mc (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
    int nr_points = indices.size ();
    for ( int i = 0; i < nr_points; ++i )
    {
      Eigen::Vector4f p(cloud_in.points[indices[i]].x, cloud_in.points[indices[i]].y, cloud_in.points[indices[i]].z, 1);
      float distance_to_plane = model_coefficients.dot(p);
      pcl::Vector4fMap pp = cloud_in.points[indices[i]].getVector4fMap();
      pp = p - mc * distance_to_plane;
    }
  }










  /** \brief Incremental Probabilistic Plane Fitting HarHar. Je ne se pais wat dat soll!? */
  class PlaneModelIncremental
  {
    public:
      PlaneModelIncremental ()
      : sum_points_(Eigen::Vector3f::Zero())
      , sum_covs_(Eigen::Matrix3f::Zero())
      , sum_weights_(0.0f), nr_points_(0)
      {}

      virtual void
      setState (const Eigen::Vector3f& sum_points, const Eigen::Matrix3f& sum_covs, const float& sum_weights, const int& nr_points = 1)
      {
        sum_points_ = sum_points;
        sum_covs_ = sum_covs;
        sum_weights_ = sum_weights;
        nr_points_ = nr_points;
      }

      float getNumberOfPoints() const { return nr_points_; };
      float getSumOfWeights () const { return sum_weights_; };

      template <typename PointT> inline void
      addPoint (const PointT& point, float weight = 1.0f)
      {
        addPoint (point.x, point.y, point.z, weight);
      }

      inline void
      addPoint (const float& x, const float& y, const float& z, float weight = 1.0f )
      {
//        if (weight < 1e-9) return;
        Eigen::Vector3f point_xyz(x, y, z);
        sum_weights_ += weight;
        sum_points_ += weight * point_xyz;
        sum_covs_ += weight * point_xyz * point_xyz.transpose();
        ++nr_points_;
      }

      template <typename PointT> inline void
      removePoint (const PointT& point, float weight = 1.0f)
      {
        removePoint (point.x, point.y, point.z, weight);
      }

      inline void
      removePoint (const float& x, const float& y, const float& z, float weight = 1.0f )
      {
//        if (weight < 1e-9) return;
        Eigen::Vector3f point_xyz(x, y, z);
        sum_weights_ -= weight;
        sum_points_ -= weight * point_xyz;
        sum_covs_ -= weight * point_xyz * point_xyz.transpose();
        --nr_points_;
      }

      inline void
      reset()
      {
        sum_points_ = Eigen::Vector3f::Zero();
        sum_covs_ = Eigen::Matrix3f::Zero();
        sum_weights_ = 0.0f;
        nr_points_ = 0;
      }




      friend std::ostream&
      operator <<(std::ostream& os, const PlaneModelIncremental& model)
      {
        os << "PlaneModelIncremental: weight=" << model.sum_weights_ << std::endl;
        return os;
      }




      inline void getPlaneCertainty(const pcl::PlaneModelIncremental& model, const float& lambda, Eigen::Matrix4f& hessian)
      {
        hessian = Eigen::Matrix4f::Zero();
        hessian.block<3,3>(0,0) -= model.sum_covs_;
        hessian(3,3) = -model.sum_weights_;
        hessian.block<1,3>(0,3) = model.sum_points_;
        hessian.block<3,1>(3,0) = model.sum_points_.transpose();
      }

      inline float getMSE(const pcl::PlaneModelIncremental& model, const pcl::PlaneModel& plane)
      {
        float error = 0.0f;
        for (int a = 0; a < 3; ++a)
          error += (2 * plane.distance / model.sum_weights_) * plane.normal(a) * model.sum_points_(a);
        for (int a = 0; a < 3; ++a)
          for (int b = 0; b < 3; ++b)
            error += plane.normal(a) * plane.normal(b) * (model.sum_covs_(a,b)/model.sum_weights_);
        error += plane.distance * plane.distance;

        return error;
      }


      /** \brief compute optimal plane (normal + distance) and return smallest eigen value */
      inline float getOptimalPlane(const pcl::PlaneModelIncremental& model, pcl::PlaneModel& plane)
      {
        Eigen::Vector3f centroid = model.sum_points_ / model.sum_weights_;

        Eigen::Matrix3f covariance;
        for (int i = 0; i < 3; ++i)
          for (int j = i; j < 3; ++j)
          {
            covariance(i,j) = model.sum_covs_(i,j)
                - centroid(i) * model.sum_points_(j)
                - centroid(j) * model.sum_points_(i)
                + centroid(i) * centroid(j) * model.sum_weights_;
            covariance(j,i) = covariance(i,j);
          }




        return plane.fromCentroidAndCovariance(centroid, covariance);
      }

      /** \brief compute optimal plane (normal + distance) _and_ hessian and return smallest eigen value */
      inline float getOptimalPlane(const pcl::PlaneModelIncremental& model, pcl::PlaneModel& plane, Eigen::Matrix4f& hessian)
      {
        float lambda = getOptimalPlane(model, plane);
        getPlaneCertainty(model, lambda, hessian);
        return lambda;
      }



    protected:

      Eigen::Vector3f sum_points_;
      Eigen::Matrix3f sum_covs_;
      float sum_weights_;
      int nr_points_;

  };


  class IncrementalPlaneFitting
  {

  };


}

#endif /* PCL_COMMON_INCREMENTAL_PLANE_FITTING_H_ */
