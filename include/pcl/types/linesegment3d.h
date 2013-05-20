/*
 * linesegment.h
 *
 *  Created on: May 31, 2012
 *      Author: elmasry
 */

#ifndef LINESEGMENT3D_H_
#define LINESEGMENT3D_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <Eigen/Dense>

#include <pcl/common/distances.h>
#include <pcl/search/flann_search.h>

#include "pcl/common/motime.h"
#include "pcl/opencv/cvutils.h"
#include "pcl/common/pcl_commons.h"
#include "pcl/common/point_common.h"
#include "pcl/utils/pointcloud_utils.h"

namespace pcl
{

  enum ContourForm
  {
    ConvexLine, ConcaveLine
  };

  template<typename PointT>
    class LineSegment3D
    {
      typedef Eigen::aligned_allocator<PointT> Alloc;
    protected:
      typename pcl::PointCloud<PointT>::Ptr cloud, inputCloud;
      Eigen::Vector3f normal, center;int id;
      std::vector<PointT, Alloc> model;
      float length;

      ContourForm contourForm;

      void
      calculateCentroid ()
      {
        Eigen::Vector3f c (0, 0, 0);
        for (size_t i = 0; i < cloud->size (); i++)
        {
          PointT& p = cloud->points[i];
          c[0] += p.x;
          c[1] += p.y;
          c[2] += p.z;
        }
        c /= (int)cloud->size ();
        center = c;
      }

      void
      calculateNormal ()
      {
        /*pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals (cloud);
         normal.Zero ();
         for (int i = 0; i < (int)normals->size (); i++)
         {
         normal += normals->points[i].getNormalVector3fMap ();
         }
         normal /= (int)normals->size ();
         normal.normalize ();*/
        normal = model[1].getVector3fMap () - model[0].getVector3fMap ();
        normal.normalize ();
      }

      pcl::PointCloud<pcl::Normal>::Ptr
      estimateNormals (typename pcl::PointCloud<PointT>::ConstPtr cloud)
      {
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setInputCloud (cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//        typename pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        ne.compute (*cloud_normals);
        return cloud_normals;
      }

      void
      calcLineModel ()
      {
        typedef Eigen::aligned_allocator<PointT> Alloc;

        if (cloud->size () < 2) throw std::runtime_error ("calcLineModel: numPoints<2");
        float len = 0;
        PointT p1, p2;
        p1 = cloud->points[0];
        p2 = cloud->points[1];

        for (size_t i = 1; i < cloud->size (); i++)
        {
          PointT& tmpP1 = cloud->points[i];
          for (size_t j = i + 1; j < cloud->size (); j++)
          {
            PointT& tmpP2 = cloud->points[j];
            float tmpLen = pcl::euclideanDistance (tmpP1, tmpP2);
            if (tmpLen > len)
            {
              len = tmpLen;
              p1 = tmpP1;
              p2 = tmpP2;
            }
          }
        }
        model.clear ();
        model.push_back (p1);
        model.push_back (p2);
        length = pcl::euclideanDistance (model[0], model[1]);
      }

    public:

      float distance_threshold;
      float max_iterations;
      Eigen::VectorXf model_coefficients;

      LineSegment3D (const LineSegment3D<PointT>& line)
      {
        LineSegment3D<PointT>::operator=(line);
      }

      LineSegment3D<PointT>&
      operator= (const LineSegment3D<PointT>& line)
      {
        cloud = line.cloud;
        inputCloud = line.inputCloud;
        center = line.center;
        normal = line.normal;
        id = line.id;
        distance_threshold = line.distance_threshold;
        max_iterations = line.max_iterations;
        //model = line.model;
        model.clear ();
        model.assign (line.model.begin (), line.model.end ());
        length = line.length;
        contourForm = line.contourForm;
        return (*this);
      }

      LineSegment3D<PointT>&
      operator+= (LineSegment3D<PointT>& line)
      {
        typename pcl::PointCloud<PointT>::Ptr tmpCloud (cloud);
        typename pcl::PointCloud<PointT>::Ptr copyIn (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud (*line.getCloud (), *copyIn);
        tmpCloud->points.insert (tmpCloud->points.end (), copyIn->points.begin (), copyIn->points.end ());
        setInputCloud (tmpCloud);
        return (*this);
      }

      bool
      operator< (const LineSegment3D<PointT>& line) const
      {
        return this->center[2] < line.center[2];
      }

      LineSegment3D (int id, float distance_threshold = 0.7, float max_iterations = 100)
      {
        this->id = id;
        this->distance_threshold = distance_threshold;
        this->max_iterations = max_iterations;
      }

      LineSegment3D (int id, const PointT& p1, const PointT& p2)
      {
        this->id = id;
        setLineModel (p1, p2);
      }

      LineSegment3D (float distance_threshold = 0.7, float max_iterations = 100)
      {
        this->distance_threshold = distance_threshold;
        this->max_iterations = max_iterations;
      }

      inline float
      getLength () const
      {
        return length;
      }

      void
      setLineModel (PointT p1, PointT p2)
      {
        std::vector<PointT, Alloc> model;
        model.push_back (p1);
        model.push_back (p2);
        setLineModel (model);
      }

      void
      setLineModel (std::vector<PointT, Alloc> model)
      {
        if (model.size () != 2) throw runtime_error ("line model should contain exactly two points");
        this->model = model;
        length = pcl::euclideanDistance (model[0], model[1]);
        center = (model[0].getVector3fMap () + model[1].getVector3fMap ()) / 2;
        calculateNormal ();
      }

      /**
       *
       */
      void
      setInputCloud (typename pcl::PointCloud<PointT>::Ptr cloud)
      {
        this->inputCloud = cloud;
        this->cloud = pcl::findAndSubtractLine<PointT> (inputCloud, distance_threshold, max_iterations);
        calculateCentroid ();
        try
        {
          calcLineModel ();
          calculateNormal ();
        } catch (exception e)
        {
          std::cerr << "Line.setInputCloud() cloud is empty\n";
        }

      }

      /**
       *
       */
      void
      setCloud (typename pcl::PointCloud<PointT>::Ptr cloud)
      {
        this->inputCloud = cloud;
        this->cloud = cloud;
        //calculate centroid
        calculateCentroid ();
        try
        {
          calcLineModel ();
        } catch (exception e)
        {
          std::cerr << "Line.setCloud() cloud is empty\n";
        }
        calculateNormal ();
      }

      void
      rotateLine (Eigen::Quaternion<float> q)
      {
        transformPointCloud (*this->inputCloud, *this->inputCloud, Eigen::Vector3f (0, 0, 0), q);
        transformPointCloud (*this->cloud, *this->cloud, Eigen::Vector3f (0, 0, 0), q);
        normal = q * normal;
        center = q * center;
        if (model.size () == 2)
        {
          model[0] = rotatePoint (model[0], q);
          model[1] = rotatePoint (model[1], q);
        }
      }

      /**
       * rotate camera to world coordinates
       */
      void
      cameraToworld ()
      {
        Eigen::Quaternion<float> q;
        q = Eigen::Quaternion<float> (-0.500398163355, 0.499999841466, -0.499601836645, 0.499999841466);
        rotateLine (q);
      }

      typename pcl::PointCloud<PointT>::Ptr
      getCloud () const
      {
        return cloud;
      }
      int
      getId () const
      {
        return id;
      }

      void
      setId (int id)
      {
        this->id = id;
      }

      Eigen::Vector3f
      getNormal () const
      {
        return normal;
      }

      inline std::vector<PointT, Alloc>
      getLineModel () const
      {
        return model;
      }

      PointT
      getCentroid () const
      {
        PointT out;
        out.x = center[0];
        out.y = center[1];
        out.z = center[2];
        return out;
      }

      Eigen::Vector3f
      getCenter () const
      {
        return center;
      }

      inline void
      logLine () const
      {
        printf ("line%d: length=%f, c=(%f,%f,%f), n=(%f,%f,%f), p1=(%f,%f,%f), p2=(%f,%f,%f)\n", this->getId (),
                this->getLength (), this->getCenter ()[0], this->getCenter ()[1], this->getCenter ()[2],
                this->getNormal ()[0], this->getNormal ()[1], this->getNormal ()[2], this->getLineModel  ()[0].x,
                this->getLineModel  ()[0].y, this->getLineModel  ()[0].z, this->getLineModel  ()[1].x, this->getLineModel  ()[1].y,
                this->getLineModel  ()[1].z);
      }

      inline void
      setContourType (ContourForm form)
      {
        this->contourForm = form;
      }

      inline ContourForm
      getContourType ()
      {
        return this->contourForm;
      }

    };

}

#define PCL_INSTANTIATE_LineSegment3D(T) template class PCL_EXPORTS pcl::LineSegment3D<T>;

#endif /* LINESEGMENT_H_ */
