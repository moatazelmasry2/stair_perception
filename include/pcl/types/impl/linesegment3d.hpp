/*
 * linesegment3d.hpp
 *
 *  Created on: Feb 4, 2013
 *      Author: elmasry
 */

#ifndef LINESEGMENT3D_HPP_
#define LINESEGMENT3D_HPP_

#include "pcl/types/linesegment3d.h"

template <typename PointT>
void pcl::LineSegment3D<PointT>::calculateCentroid ()
{
  Eigen::Vector3f c (0, 0, 0);
  for (size_t i = 0; i < cloud->size (); i++)
  {
    PointT& p = cloud->points[i];
    c[0] += p.x;
    c[1] += p.y;
    c[2] += p.z;
  }
  c /= (int) cloud->size ();
  center = c;
}

template <typename PointT>
void pcl::LineSegment3D<PointT>::calculateNormal ()
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

template <typename PointT>
void pcl::LineSegment3D<PointT>::calcLineModel ()
{
  typedef Eigen::aligned_allocator<PointT> Alloc;

  if (cloud->size () < 2)
    throw std::runtime_error ("calcLineModel: numPoints<2");
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

template <typename PointT>
pcl::LineSegment3D<PointT>::LineSegment3D (const LineSegment3D<PointT>& line)
{
  LineSegment3D<PointT>::operator=(line);
}

template <typename PointT>
pcl::LineSegment3D<PointT>&
pcl::LineSegment3D<PointT>::operator= (const LineSegment3D<PointT>& line)
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

template <typename PointT>
pcl::LineSegment3D<PointT>&
pcl::LineSegment3D<PointT>::operator+= (LineSegment3D<PointT>& line)
{
  typename pcl::PointCloud<PointT>::Ptr tmpCloud (cloud);
  typename pcl::PointCloud<PointT>::Ptr copyIn (new pcl::PointCloud<PointT>);
  pcl::copyPointCloud (*line.getCloud (), *copyIn);
  tmpCloud->points.insert (tmpCloud->points.end (), copyIn->points.begin (), copyIn->points.end ());
  setInputCloud (tmpCloud);
  return (*this);
}

template <typename PointT>
bool pcl::LineSegment3D<PointT>::operator< (const LineSegment3D<PointT>& line) const
{
  return this->center[2] < line.center[2];
}

template <typename PointT>
pcl::LineSegment3D<PointT>::LineSegment3D (int id, float distance_threshold, float max_iterations )
{
  this->id = id;
  this->distance_threshold = distance_threshold;
  this->max_iterations = max_iterations;
}

template <typename PointT>
pcl::LineSegment3D<PointT>::LineSegment3D (int id, const PointT& p1, const PointT& p2)
{
  this->id = id;
  setLineModel (p1, p2);
}

template <typename PointT>
pcl::LineSegment3D<PointT>::LineSegment3D (float distance_threshold, float max_iterations)
{
  this->id = 0;
  this->distance_threshold = distance_threshold;
  this->max_iterations = max_iterations;
}


template <typename PointT>
void pcl::LineSegment3D<PointT>::setLineModel (PointT p1, PointT p2)
{
  std::vector<PointT, Eigen::aligned_allocator<PointT> > model;
  model.push_back (p1);
  model.push_back (p2);
  setLineModel (model);
}

template <typename PointT>
void pcl::LineSegment3D<PointT>::setLineModel (std::vector<PointT, Eigen::aligned_allocator<PointT> > model)
{
  if (model.size () != 2)
    throw runtime_error ("line model should contain exactly two points");
  this->model = model;
  length = pcl::euclideanDistance (model[0], model[1]);
  center = (model[0].getVector3fMap () + model[1].getVector3fMap ()) / 2;
  calculateNormal ();
}

/**
 *
 */
template <typename PointT>
void pcl::LineSegment3D<PointT>::setInputCloud (typename pcl::PointCloud<PointT>::Ptr cloud)
{
  this->inputCloud = cloud;
  this->cloud = pcl::findAndSubtractLine<PointT> (inputCloud, distance_threshold, max_iterations);
  calculateCentroid ();
  try
  {
    calcLineModel ();
    calculateNormal ();
  }
  catch (exception e)
  {
    std::cerr << "Line.setInputCloud() cloud is empty\n";
  }

}

/**
 *
 */
template <typename PointT>
void pcl::LineSegment3D<PointT>::setCloud (typename pcl::PointCloud<PointT>::Ptr cloud)
{
  this->inputCloud = cloud;
  this->cloud = cloud;
  //calculate centroid
  calculateCentroid ();
  try
  {
    calcLineModel ();
  }
  catch (exception e)
  {
    std::cerr << "Line.setCloud() cloud is empty\n";
  }
  calculateNormal ();
}

template <typename PointT>
void pcl::LineSegment3D<PointT>::rotateLine (Eigen::Quaternion<float> q)
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
template <typename PointT>
void pcl::LineSegment3D<PointT>::cameraToworld ()
{
  Eigen::Quaternion<float> q;
  q = Eigen::Quaternion<float> (-0.500398163355, 0.499999841466, -0.499601836645, 0.499999841466);
  rotateLine (q);
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr pcl::LineSegment3D<PointT>::getCloud () const
{
  return cloud;
}

template <typename PointT>
inline std::vector<PointT, Eigen::aligned_allocator<PointT> > pcl::LineSegment3D<PointT>::getModel () const
{
  return model;
}

template <typename PointT>
PointT pcl::LineSegment3D<PointT>::getCentroid () const
{
  PointT out;
  out.x = center[0];
  out.y = center[1];
  out.z = center[2];
  return out;
}


template<typename PointT>
inline void pcl::LineSegment3D<PointT>::logLine () const
{
  printf ("line%d: length=%f, c=(%f,%f,%f), n=(%f,%f,%f), p1=(%f,%f,%f), p2=(%f,%f,%f)\n", this->getId (),
      this->getLength (), this->getCenter ()[0], this->getCenter ()[1], this->getCenter ()[2], this->getNormal ()[0],
      this->getNormal ()[1], this->getNormal ()[2], this->getModel ()[0].x, this->getModel ()[0].y,
      this->getModel ()[0].z, this->getModel ()[1].x, this->getModel ()[1].y, this->getModel ()[1].z);
}

#endif /* LINESEGMENT3D_HPP_ */
