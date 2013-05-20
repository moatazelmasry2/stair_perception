/*
 * planesegmentation.h
 *
 *  Created on: Jun 22, 2012
 *      Author: elmasry
 *
 *  Original Code by Dirk Holz
 */

#ifndef PLANESEGMENTATION_H_
#define PLANESEGMENTATION_H_

#include <map>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/point_cloud_handlers.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ros/conversions.h>

#include <pcl/apps/fast_meshing.h>

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/angles.h>
#include <Eigen/Core>
#include <pcl/common/mesh_utilities.h>

//#include <pcl/surface/plane_detection.h>
//#include <pcl/surface/vtk_smoother.h>
#include <pcl/surface/surface_functions.h>
//#include <pcl/surface/ear_clipping.h>

#include <pcl/kdtree/fixed_neighbors.h>

#include <pcl/segmentation/region_segmentation.h>
#include <pcl/common/color.h>

#include <pcl/features/normal_3d.h>

#include "pcl/pcl_macros.h"
#include "pcl/common/math.h"
#include "pcl/common/pcl_commons.h"
#include "pcl/common/time.h"
#include "pcl/common/point_common.h"
#include "pcl/types/plane3d.h"
#include "pcl/types/plane3d_utils.h"
#include "pcl/utils/pointcloud_utils.h"
#include "pcl/histogram/planehistogram.h"

namespace pcl
{

  template<typename PointT, typename PointOut>
  class PCL_EXPORTS PlaneSegmentation : public pcl::PCLBase<PointT>
  {
      typedef typename pcl::PointCloud<PointOut>::Ptr PointOutPtr;
      typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
      typedef Eigen::aligned_allocator<PointOut> Alloc;
      typedef Eigen::aligned_allocator<Plane3D<PointOut> > PlaneAlloc;
      typedef Eigen::aligned_allocator<int> IntAlloc;
      typedef std::vector<Plane3D<PointOut>, Alloc> Plane3DVector;
      typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > PointsVector;

      using pcl::PCLBase<PointT>::input_;

    protected:
      bool validInput;
      Plane3DVector planes, rawPlanes, walls;

      pcl::PolygonMesh::Ptr mesh;
      typename pcl::PointCloud<PointOut>::Ptr cloud2;

      float angle;

    public:
      PointsVector discardedPoints;
    protected:

      void calculatePlanes (std::vector<std::vector<int, IntAlloc>, IntAlloc> point_to_cluster, Plane3DVector& planes)
      {
        int nr_points = (int) cloud2->size ();
        std::map<int, PointOutPtr> planesMap;
        typename std::map<int, PointOutPtr>::iterator it;

        planes.clear ();

        for (int i = 0; i < nr_points; ++i)
        {
          if (point_to_cluster[i].size () > 1)
          {
//            discardedPoints.push_back (cloud2->points[i]);
            continue;
          }

          for (size_t j = 0; j < point_to_cluster[i].size (); j++)
          {
            //skip points that belong to multiple planes

            it = planesMap.find (point_to_cluster[i][j]);

            if (it != planesMap.end ())
            {
              //if plane is found
              PointOutPtr tmpCloud = it->second;
              tmpCloud->push_back (cloud2->points[i]);
            }
            else
            {
              //if no plane found, create one
              PointOutPtr tmpCloud (new pcl::PointCloud<PointOut>);
              tmpCloud->push_back (cloud2->points[i]);
              planesMap[point_to_cluster[i][j]] = tmpCloud;
            }
          }
        }

        int count = 0;
        for (it = planesMap.begin (); it != planesMap.end (); it++)
        {
          //printf("");
          //Plane3D<PointOut> plane = Plane3DFactory<PointOut>::createPlane3D(it->second, count++);
          Plane3D<PointOut> plane (count++);
          plane.setInputCloud (it->second);
          planes.push_back (plane);
        }
      }

      void mergePlanes2 (Plane3DVector& planes)
      {
        pcl::PlaneHistogram<pcl::Plane3D<PointOut>, PointOut> horHist (2), vertHist (0);
        for (typename Plane3DVector::iterator it = planes.begin (); it != planes.end (); it++)
        {
          Plane3D<PointOut>& plane = *it;
          if (plane.isTread () )
          {
            horHist.addPlane (plane);
          }
          else if (plane.isRiser () && plane.isMinimalSize())
          {
            vertHist.addPlane (plane);
          }
        }
        float threshold = 0.07, threshold2 = 0.1;
        horHist.setStepsize (threshold);
        vertHist.setStepsize (threshold);
        horHist.maxDistanceThreshold = threshold2;
        vertHist.maxDistanceThreshold = threshold2;
        horHist.compute ();
        vertHist.compute ();
        //Plane3DVector hPlanes = horHist.getOutplanes (), vPlanes = vertHist.getOutplanes ();
        std::vector<Plane3D<PointOut>, PlaneAlloc> hPlanes = horHist.getOutplanes (), vPlanes =
            vertHist.getOutplanes ();
        planes.clear ();
        planes.insert (planes.end (), hPlanes.begin (), hPlanes.end ());
        planes.insert (planes.end (), vPlanes.begin (), vPlanes.end ());
      }

      void savePlanes (Plane3DVector& planes)
      {
        pcl::PointCloud<PointOut> outCloud;
        for (size_t i = 0; i < planes.size (); i++)
        {
          pcl::Plane3D<PointOut>& plane = planes[i];
          char buf[10];
          sprintf (buf, "plane%d.pcd", plane.getId ());
          typename pcl::PointCloud<PointOut>::ConstPtr ptr (plane.getInputCloud ());
          pcl::PointCloud<PointOut> tmpCloud = *ptr;
          pcl::colorCloud (tmpCloud, pcl::color::getRandomColor (0.1, 2.9));
          outCloud += tmpCloud;

          pcl::PointCloud<PointT> tmpCloud2;
          pcl::copyPointCloud (tmpCloud, tmpCloud2);
          //pcl::writeCloud (tmpCloud2, buf);
        }
        char buf2[50];
        sprintf (buf2, "%ld.pcd", pcl::getTimeMs ());
        pcl::io::savePCDFileASCII (buf2, outCloud);
      }

      void removeSmallPlanes (Plane3DVector& planes, int minNumPoints = 400)
      {
        pcl::PointCloud<PointOut> outCloud;
        Plane3DVector tmpPlanes;
        for (size_t i = 0; i < planes.size (); i++)
        {
          pcl::Plane3D<PointOut>& plane = planes[i];
          if (plane.getCloud ()->size () > (size_t) minNumPoints)
          {
            tmpPlanes.push_back (plane);
          }
        }
        planes.clear ();
        planes.assign (tmpPlanes.begin (), tmpPlanes.end ());
      }

      void removeNonHorVertPlanes (Plane3DVector& planes)
      {
        pcl::PointCloud<PointOut> outCloud;
        Plane3DVector tmpPlanes;
        for (size_t i = 0; i < planes.size (); i++)
        {
          pcl::Plane3D<PointOut>& plane = planes[i];
          if ( (plane.isHorizontal () || plane.isVertical ()))
          {
            tmpPlanes.push_back (plane);
          }
        }
        planes.clear ();
        planes.assign (tmpPlanes.begin (), tmpPlanes.end ());
      }
      /**
       * Removes planes that are neither horizontal nor vertical, or those that are not vertical to side walls, i.e. not part of a staircase
       */
      void removeNonstepPlanes (Plane3DVector& planes)
      {
        pcl::PointCloud<PointOut> outCloud;
        Plane3DVector tmpPlanes;
        for (size_t i = 0; i < planes.size (); i++)
        {
          pcl::Plane3D<PointOut>& plane = planes[i];
          if ( (plane.isHorizontal () || plane.isVertical ())
              && fabs (90 - pcl::getAngle (plane.normal, Eigen::Vector3f (0, 1, 0))) < 14)
          {
            tmpPlanes.push_back (plane);
          }
        }
        planes.clear ();
        planes.assign (tmpPlanes.begin (), tmpPlanes.end ());
      }

      /**
       * Roatete points, normals, centre etc...
       * \param angle: in degrees
       */
      inline void rotatePlanes (Plane3DVector& planes, double angle, Eigen::Vector3f axis)
      {
        std::cout << "Rotating everything with " << angle << " degrees on axis: " << axis[0] << "," << axis[1] << ","
            << axis[2] << std::endl;
        Quaternion<float> q;
        q = AngleAxis<float> (angle * M_PI / 180, axis);

        rotatePlanes (planes, q);
      }

      /**
       * Roatete points, normals, centre etc...
       * \param angle: in degrees
       */
      inline void rotatePlanes (Plane3DVector& planes, Quaternion<float> q)
      {

        for (size_t i = 0; i < planes.size (); i++)
        {
          Plane3D<PointOut>& plane = planes[i];
          const typename pcl::PointCloud<PointOut>::ConstPtr input = plane.getInputCloud ();
          typename pcl::PointCloud<PointOut>::Ptr outPtr (new pcl::PointCloud<PointOut>);
          plane.setNormal (q * plane.getNormal ());
          plane.setCenter (q * plane.getCenter ());
          pcl::copyPointCloud (*input, *outPtr);
          pcl::rotatePointCloud (*outPtr, q);
          const typename pcl::PointCloud<PointOut>::ConstPtr constCloud (outPtr);
          //printf("planeId=%d, numPoints=%d\n", plane.getId(), (int)outPtr->points.size() );
          plane.setInputCloud (outPtr, false);
        }
      }

      inline void rotateNormals (std::vector<Eigen::Vector3f>& normals, const float angle, const Eigen::Vector3f axis)
      {
        Quaternion<float> q;
        q = AngleAxis<float> (angle * M_PI / 180, axis);
        for (size_t i = 0; i < normals.size (); i++)
        {
          normals[i] = q * normals[i];
        }
      }

      /**
       * \param normals to the plane. input param
       * \param outAngle rotation angle
       * \param outAxis to rotate on
       * \throws exception if number of input normals = 0
       */
      inline void findRotation (const std::vector<Eigen::Vector3f>& normals, float& outAngle, Eigen::Vector3f& outAxis)
      {
        if (normals.size () == 0)
          throw std::runtime_error ("rotateEverything: No planes given to rotate. exit\n");

        Eigen::Vector3f zeroVec (0, 0, 1), sumNormals (0, 0, 0);
        float sumAngle = 0, sumWeights = 0, minAngle = 360, maxAngle = 0;
        std::vector<float> angles;

        //calculates the mean of all normals and the sum of angles related to zerovector
        //Target: calculates weighted average of normals
        //find the min angle
        for (size_t i = 0; i < normals.size (); i++)
        {
          float angle = calcAngle3DDegree (zeroVec, normals[i]);
          angles.push_back (angle);
          minAngle = angle < minAngle ? angle : minAngle;
          maxAngle = angle > maxAngle ? angle : maxAngle;
        }

        //calculates wights of normals, angles and find the weighted average of angle and normals
        for (size_t i = 0; i < normals.size (); i++)
        {
          float weight = pcl::translateNumberRange (angles[i], minAngle, maxAngle, 1.0f, 0.0f);
          if (weight < 0.8)
            continue;
          sumWeights += weight;
          sumNormals += normals[i] * weight;
          sumAngle += angles[i] * weight;
        }

        Eigen::Vector3f avWeightedNormal = sumNormals / sumWeights;
        //printf("avgWeightedNormal=(%f,%f,%f)\n", avWeightedNormal[0], avWeightedNormal[1], avWeightedNormal[2]);
        outAxis = avWeightedNormal.cross (zeroVec);
        outAngle = sumAngle / sumWeights;
        outAxis.normalize ();
      }

      inline void rotateEverything2 (Plane3DVector& planes)
      {
        std::vector<Eigen::Vector3f> normals;
        for (size_t i = 0; i < planes.size (); i++)
        {
          normals.push_back (planes[i].getNormal ());
        }

        Quaternion<float> q;
        //logNormals(normals);
        for (size_t i = 0; i < 1; i++)
        {
          float angle;
          Eigen::Vector3f axis (0, 0, 0);
          findRotation (normals, angle, axis);
          //printf ("angle=%f, axis=(%f,%f,%f)\n", angle, axis[0], axis[1], axis[2]);
          rotateNormals (normals, angle, axis);
          if (i == 0)
          {
            q = AngleAxis<float> (angle * M_PI / 180, axis);
          }
          else
          {
            Quaternion<float> q2;
            q2 = AngleAxis<float> (angle * M_PI / 180, axis);
            q *= q2;
          }
          //logNormals (normals);
        }
        rotQuaternion = q;
        Eigen::Matrix3f mat = q.toRotationMatrix ();
        angle = acos (mat (0, 0)) * 180 / M_PI;
        rotatePlanes (planes, q);
      }

      void logPlanes (Plane3DVector& planes)
      {
        printf ("numPlanes=%d\n", (int) planes.size ());
        Eigen::Vector3f zeroVec (0, 0, 1);
        for (size_t i = 0; i < planes.size (); i++)
        {
          Plane3D<PointOut>& plane = planes[i];
          plane.logPlane ();
          //double angle = calcAnglebetweenVectors (zeroVec, plane.normal);
          //double angle = calcAngle3D (zeroVec, plane.getNormal()) * 180 / M_PI;
          //printf ("plane%d, center=(%f,%f,%f),normal=(%f,%f,%f), angle=%f\n", plane.get, plane.center[0],
          //        plane.center[1], plane.center[2], plane.normal[0], plane.normal[1], plane.normal[2], angle);
        }
      }

      struct Comparator
      {
          int compIndex;
          Comparator (int compIndex)
          {
            this->compIndex = compIndex;
          }
          bool operator() (const pcl::Plane3D<PointOut>& plane1, const pcl::Plane3D<PointOut>& plane2)
          {
            return plane1.getCenter ()[compIndex] < plane2.getCenter ()[compIndex] ? true : false;
          }
      };

      void sortPlanes (Plane3DVector& planes)
      {

        Comparator comp (2);
        std::sort (planes.begin (), planes.end (), comp);
      }

      void addDiscardedPoints(Plane3DVector& vec) {
//        for (size_t i = 0; i < )
      }
      void computePlanes (Plane3DVector& vec)
      {
        if (!validInput)
          return;
        //int nr_polygons = (int)mesh->polygons.size ();

        /// retrieve point coordinates //////////////////////////////////////////////
        typename pcl::PointCloud<PointT>::Ptr cloud_xyz (new pcl::PointCloud<PointT>);
        pcl::fromROSMsg (mesh->cloud, *cloud_xyz);

        int nr_points = (int) cloud_xyz->points.size ();

        /// construct initial vertex-vertex-list ////////////////////////////////////
        pcl::NeighborhoodVectorPtr vertex_vertex_lists (new pcl::NeighborhoodVector);
        bool ignore_duplicates = true;
        pcl::getVertexVertexList (*mesh, *vertex_vertex_lists, ignore_duplicates);

        /// extending vertex neighborhood to the k-ring neighborhood ////////////////
        bool extend_to_k_ring_neighborhood = true;
        if (extend_to_k_ring_neighborhood)
        {
          const int k_ring_neighborhood_size = 10;
          pcl::NeighborhoodVectorPtr ring_neighborhood_temp (new pcl::NeighborhoodVector);
          pcl::computeRingNeighborhood (*vertex_vertex_lists, *ring_neighborhood_temp, k_ring_neighborhood_size);

          int nr_vertices = (int) ring_neighborhood_temp->size ();
          int sum_neighbors = 0;
          for (int i = 0; i < nr_vertices; ++i)
            sum_neighbors += ring_neighborhood_temp->at (i).size ();
          //float mean_neighbors = (float)sum_neighbors / (float)nr_vertices;

          sum_neighbors = 0;
          nr_vertices = (int) vertex_vertex_lists->size ();
          for (int i = 0; i < nr_vertices; ++i)
            sum_neighbors += vertex_vertex_lists->at (i).size ();
          //mean_neighbors = (float)sum_neighbors / (float)nr_vertices;

          vertex_vertex_lists.swap (ring_neighborhood_temp);
        }

        //std::ofstream file_output ("neighborhood_temp.txt", std::ios::trunc);
        //if (file_output.is_open ())
        //{
        //  for (int point = 0; point < (int)vertex_vertex_lists->size (); ++point)
        //  {
        //    file_output << point << "     ";
        //    for (int neighbor = 0; neighbor < (int)vertex_vertex_lists->at (point).size (); ++neighbor)
        //      file_output << (*vertex_vertex_lists)[point][neighbor] << ' ';
        //    file_output << std::endl;
        //  }
        //  file_output.close ();
        //  file_output.clear ();
        //}

        /// compute normals in mesh /////////////////////////////////////////////////
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

//        bool approximate_normals = true;
//        if (approximate_normals)
//        {
        pcl::surface::computeNormalsAreaWeighted (*cloud_xyz, mesh->polygons, *cloud_normals);
        //    pcl::surface::computeNormalsAreaWeightedInverse(*cloud_xyz, mesh->polygons, *cloud_normals);
//        } else
//        {
//          pcl::NormalEstimation<PointT, pcl::Normal> ne;
//          typename pcl::FixedNeighbors<PointT>::Ptr ne_search (new pcl::FixedNeighbors<PointT>);
//          ne_search->setInputCloud (cloud_xyz);
//          ne_search->setNeighborhood (vertex_vertex_lists);
//          ne.setInputCloud (cloud_xyz);
//          ne.setSearchMethod (ne_search);
//          ne.setKSearch (25);
//          ne.compute (*cloud_normals);
//        }
        typename pcl::PointCloud<PointOut>::Ptr cloud2 (new pcl::PointCloud<PointOut>);
        pcl::concatenateFields (*cloud_xyz, *cloud_normals, *cloud2);
        int nr_points_with_normals = (int) cloud2->points.size ();
        nr_points = nr_points_with_normals;

        bool getMeshColors = true;
        if (getMeshColors)
        {
          const int idx_rgb = pcl::getFieldIndex (mesh->cloud, "rgb");
          if (idx_rgb != -1)
            pcl::surface::makeNormalColors (*cloud2);
          else
          {
            for (int i = 0; i < nr_points_with_normals; ++i)
            {
              cloud2->points[i].r = 255;
              cloud2->points[i].g = 255;
              cloud2->points[i].b = 255;
            }
          }
        }

        /// setup search tree ///////////////////////////////////////////////////////
        typename pcl::FixedNeighbors<PointOut>::Ptr search (new pcl::FixedNeighbors<PointOut>);
        search->setInputCloud (cloud2);
        search->setNeighborhood (vertex_vertex_lists);
        //  pcl::KdTreeFLANN<PointOut>::Ptr search(new pcl::KdTreeFLANN<PointOut>);
        //  search->setInputCloud(cloud);

        /// setup noise model ///////////////////////////////////////////////////////
        typename pcl::QuadraticIsotropicNoise<PointOut>::Ptr noise_model (new pcl::QuadraticIsotropicNoise<PointOut>);
        noise_model->setModelHolz ();
        noise_model->setInputCloud (cloud2);
        /// compute/estimate curvature /////////////////////////////////////////////
        //  pcl::surface::computeApproximateCurvature(*cloud, *vertex_vertex_lists);
        //  float mean_curvature = 0.0f;

        /// mesh SEGMENTATION by region growing /////////////////////////////////////
        std::vector<pcl::PointIndices> clusters;
        //float distance_tolerance = 0.1f;
        //double angle_tolerance = pcl::deg2rad (15.0f);
        //  double angle_tolerance = pcl::deg2rad(25.0f);
        //unsigned int min_points_per_cluster = 25;
        //unsigned int max_points_per_cluster = std::numeric_limits<int>::max ();
        pcl::segmentation::LastNormalRegionSegmentation<PointOut> extract;
//        pcl::segmentation::InitialNormalRegionSegmentation<PointOut> extract;
        //pcl::segmentation::ProbabilisticPlaneRegionSegmentation<PointOut> extract;
        //pcl::segmentation::ApproximatePlaneRegionSegmentation<PointOut> extract;

        extract.setSearchMethod (search);
        extract.setNoiseModel (noise_model);
        extract.setInputCloud (cloud2);
        extract.setMinClusterSize (100);
        extract.setMaxClusterSize (std::numeric_limits<int>::max ());
        extract.setAngleTolerance (pcl::deg2rad (4.0f));
        extract.allowMultiAssignments (false);
        extract.makeCompleteSegmentations (false);
        extract.extract (clusters);

        int nr_clusters = (int) (clusters.size ());

        /// colorize multiply connected vertices ///////////////////////////////////
        std::vector<std::vector<int, IntAlloc>, IntAlloc> point_to_cluster;
        point_to_cluster.resize (nr_points);
        for (int i = 0; i < nr_clusters; ++i)
        {
          int nr_points_cluster = (int) clusters[i].indices.size ();

          for (int j = 0; j < nr_points_cluster; ++j)
            point_to_cluster[clusters[i].indices[j]].push_back (i);
        }

        std::vector<pcl::PointIndices> corrected_clusters;
        corrected_clusters.resize (nr_clusters);
        for (int i = 0; i < nr_clusters; ++i)
          corrected_clusters[i] = clusters[i];

        for (int i = 0; i < nr_points; ++i)
        {
          if (point_to_cluster[i].size () == 0)
          {
            const pcl::Neighborhood& point_neighborhood = (*vertex_vertex_lists)[i];
            for (int j = 0; j < (int) point_neighborhood.size (); ++j)
            {
              if (point_to_cluster[point_neighborhood[j]].size () > 0)
              {
                point_to_cluster[i].push_back (point_to_cluster[point_neighborhood[j]][0]);
                corrected_clusters[point_to_cluster[point_neighborhood[j]][0]].indices.push_back (i);
              }
            }
          }
        }

        /// colorize segments //////////////////////////////////////////////////////
        pcl::segmentation::colorizeSegments (*cloud2, clusters);
//        pcl::io::savePCDFileASCII("colorized_clusters.pcd", *cloud2);

        //colorize points belonging to no clusters or to multiple clusters
        int nr_points_with_multiple_assignments = 0;
        int nr_points_with_exactly_one_assignment = 0;
        int nr_points_with_no_assignment = 0;
        for (int i = 0; i < nr_points; ++i)
        {
          if (point_to_cluster[i].size () == 0)
          {
            cloud2->points[i].rgb = pcl::color::getColorFloatFromRGB (0, 0, 0);
            ++nr_points_with_no_assignment;
          }
          else if (point_to_cluster[i].size () == 1)
          {
            ++nr_points_with_exactly_one_assignment;
          }
          else if (point_to_cluster[i].size () > 1)
          {
            cloud2->points[i].rgb = pcl::color::getColorFloatFromRGB (255, 0, 0);
            ++nr_points_with_multiple_assignments;
          }
        }
//        pcl::io::savePCDFileASCII("colorized_clusters2.pcd", *cloud2);

        this->cloud2 = cloud2;
        calculatePlanes (point_to_cluster, vec);
      }

      void findWalls ()
      {
        for (size_t i = 0; i < rawPlanes.size (); i++)
        {
          const Plane3D<PointOut>& plane = rawPlanes[i];
          if (::pcl::utils::isWall (plane))
          {
            walls.push_back (plane);
          }
        }
      }

      Plane3DVector mergeWalls (const Plane3DVector& walls)
      {
        pcl::PlaneHistogram<pcl::Plane3D<PointOut>, PointOut> hist (1);
        for (size_t i = 0; i < walls.size (); i++)
        {
          hist.addPlane (walls[i]);
        }
        float threshold = 0.04, threshold2 = 0.1;
        hist.setStepsize (threshold);
        hist.maxDistanceThreshold = threshold2;
        hist.compute ();
        std::vector<Plane3D<PointOut>, PlaneAlloc> tPlanes = hist.getOutplanes ();

        Plane3DVector outPlanes;
        outPlanes.insert (outPlanes.end (), tPlanes.begin (), tPlanes.end ());
        return outPlanes;
      }

      void logNormals (std::vector<Eigen::Vector3f> normals)
      {
        printf ("logging normals=%d\n", (int) normals.size ());
        for (size_t i = 0; i < normals.size (); i++)
        {
          Eigen::Vector3f n = normals[i];
          printf ("n=(%f,%f,%f), angles to earth normals=(%f,%f,%f)", n[0], n[1], n[2],
              pcl::calcAngle3DDegree (n, Eigen::Vector3f (1, 0, 0)),
              pcl::calcAngle3DDegree (n, Eigen::Vector3f (0, 1, 0)),
              pcl::calcAngle3DDegree (n, Eigen::Vector3f (0, 0, 1)));
          printf ("\n");
        }
      }

    public:

      Quaternion<float> rotQuaternion;
      /** \brief Provide a pointer to the input dataset
       * \param cloud the const boost shared pointer to a PointCloud message
       */
      virtual inline void setInputCloud (const PointCloudConstPtr &cloud)
      {
        validInput = true;
        input_ = cloud;
        planes.clear ();
        rawPlanes.clear ();
        walls.clear ();

        pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);

        pcl::FastMeshing<PointT, PointOut> mesh_builder;
        std::vector<int> original_indices;
        mesh_builder.mesh (input_, mesh, original_indices);

        // Computing normals
        typename pcl::PointCloud<PointT>::Ptr cloud_copy (new pcl::PointCloud<PointT> ());
        try
        {
          pcl::surface::getCloud (*mesh, *cloud_copy);
        }
        catch (exception e)
        {
          validInput = false;
          return;
        }
        typename pcl::PointCloud<PointOut>::Ptr cloud_mesh (new pcl::PointCloud<PointOut> ());
        pcl::surface::computeNormalsAreaWeighted (*cloud_copy, mesh->polygons, *cloud_mesh);
        pcl::concatenateFields (*cloud_mesh, *cloud_copy, *cloud_mesh);

        bool colorize_wrt_normals = false;
        if (colorize_wrt_normals)
          pcl::surface::makeNormalColors (*cloud_mesh);
        pcl::toROSMsg (*cloud_mesh, mesh->cloud);
//        pcl::io::savePolygonFileVTK ("temp_mesh_raw.vtk", *mesh);
//        pcl::io::savePCDFileASCII("temp_mesh_raw.pcd", *cloud_mesh);

        bool smooth_mesh = true;
        if (smooth_mesh)
        {
          std::vector<std::vector<int> > vertex_list;
          pcl::getVertexVertexList (*mesh, vertex_list);

          // Bilateral Filtering!!!!!
          unsigned int filter_runs = 2;
          typename pcl::PointCloud<PointOut>::Ptr cloud_filtered (new pcl::PointCloud<PointOut> (*cloud_mesh));
          for (unsigned int run = 0; run < filter_runs; ++run)
          {
            typename pcl::PointCloud<PointOut>::Ptr cloud_temp (new pcl::PointCloud<PointOut> ());
            pcl::surface::smoothBilateralNEW (*cloud_filtered, vertex_list, *cloud_temp);
            cloud_filtered.swap (cloud_temp);
          }

          for (size_t i = 0; i < cloud_mesh->points.size (); ++i)
          {
            cloud_filtered->points[i].rgb = cloud_mesh->points[i].rgb;
            cloud_filtered->points[i].id = cloud_mesh->points[i].id;
          }

//          pcl::io::savePCDFileASCII("filtered_cloud.pcd", *cloud_filtered);

          float a2 = -0.243897e-3, a1 = 0.486625e-2, a0 = -0.7691149 * 1e-3;
          for (unsigned int i = 0; i < cloud_mesh->points.size (); ++i)
          {
            PointOut& pt = cloud_mesh->points[i];
            const float d = pt.getVector3fMap ().norm ();
            const float sigma = a2 * d * d + a1 * d + a0;
            //const float sq_sigma = sigma * sigma;
            pt.curvature = sigma;  //sq_sigma;
          }

          if (colorize_wrt_normals)
            pcl::surface::makeNormalColors (*cloud_filtered);
          pcl::toROSMsg (*cloud_filtered, mesh->cloud);
        }
        this->mesh = mesh;
//        pcl::io::savePolygonFileVTK ("filtered_mesh.vtk", *mesh);

      }

      void compute ()
      {
        discardedPoints.clear ();
        Plane3DVector vec;
        computePlanes (vec);

        //printf ("initial numPlanes=%d\n", (int)vec.size ());
        rawPlanes.clear ();
        rawPlanes.assign (vec.begin (), vec.end ());
        sortPlanes (vec);

//        printf ("beforeRotation\n");
//        logPlanes (vec);

        bool rotate = true;
        if (rotate && vec.size () > 0)
        {
          rotateEverything2 (vec);
        }

//        printf ("after rotation\n");
//        logPlanes (vec);

        bool wallsFind = true;
        if (wallsFind)
        {
          findWalls ();
          walls = mergeWalls (walls);
        }

        bool rmNonHorVert = true;
        if (rmNonHorVert && vec.size () > 0)
          removeNonHorVertPlanes (vec);
//        printf ("afterRemovenonHorVert=%d\n", vec.size());
//        logPlanes (vec);
        bool merge = true;
        if (merge && vec.size () > 0)
        {
//          printf ("Planes before Merge=%d\n", (int)vec.size ());
          mergePlanes2 (vec);
//          printf ("numPlanesAfterMerge=%d\n", (int)vec.size ());
//          logPlanes (vec);
        }

        //sortPlanes (vec);
        //printf ("planes after merge\n");
        //logPlanes (vec);
        bool removesmallplanes = false;
        if (removesmallplanes && vec.size () > 0)
          removeSmallPlanes (vec);
        //printf ("numPlanes After removeSmallPlanes=%d\n", (int)vec.size ());

        /*bool postProcess = true;
         if (postProcess)
         postprocessPlanes (vec);*/

        sortPlanes (vec);
//        logPlanes (vec);

        addDiscardedPoints(vec);
        planes.clear ();
        planes.assign (vec.begin (), vec.end ());

//        float black = pcl::generateColor (5, 5, 5);
//        float red = pcl::generateColor (255, 0, 0);
//
//        typename pcl::PointCloud<PointOut>::Ptr myoutCloud (new pcl::PointCloud<PointOut>);
//
//        for (size_t i = 0; i < vec.size (); i++)
//        {
//          Plane3D<PointOut> plane = vec[i];
//          typename pcl::PointCloud<PointOut>::Ptr cloud = plane.getCloud ();
//          pcl::colorCloud (*cloud, black);
//          for (size_t j = 0; j < plane.convexHull.size (); j++)
//          {
//            plane.convexHull[j].rgb = red;
//          }
//          cloud->insert (cloud->end (), plane.convexHull.begin (), plane.convexHull.end ());
//          myoutCloud->insert (myoutCloud->end (), cloud->begin (), cloud->end ());
//          char f[50];
//          sprintf (f, "p_%d.pcd", (int) i);
//          pcl::io::savePCDFileBinary (f, *cloud);
//        }
//        pcl::io::savePCDFileBinary ("planes.pcd", *myoutCloud);
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//        typename pcl::PointCloud<PointOut>::Ptr bordersCloud (new pcl::PointCloud<PointOut>);
//        for (size_t i = 0; i < planes.size (); i++)
//        {
//          Plane3D<PointOut> plane = planes[i];
//          typename pcl::PointCloud<PointOut>::Ptr c = plane.getCloud ();
//          pcl::colorCloud (*c, black);
//          std::vector<PointOut> bbox;
//          bbox.push_back (plane.bbox.topLeft);
//          bbox.push_back (plane.bbox.topRight);
//          bbox.push_back (plane.bbox.bottomRight);
//          bbox.push_back (plane.bbox.bottomLeft);
//          typename pcl::PointCloud<PointOut>::Ptr edge;
//          for (size_t j = 0; j < bbox.size () - 1; j++)
//          {
//            edge = reconstructLineSegment (bbox[j], bbox[j + 1], 0.01f);
//            colorCloud (*edge, red);
//            concatePointClouds (*edge, *c);
//          }
//          edge = reconstructLineSegment (bbox[3], bbox[0], 0.01f);
//          colorCloud (*edge, red);
//          concatePointClouds (*edge, *c);
//          concatePointClouds (*c, *bordersCloud);
//        }
//        pcl::io::savePCDFile ("borders_cloud.pcd", *bordersCloud);

//        printf ("planeSegmentation final result. numplanes=%d", (int)planes.size ());
//        logPlanes(planes);
      }

      inline typename pcl::PointCloud<PointOut>::Ptr getOutputCloud ()
      {
        typename pcl::PointCloud<PointOut>::Ptr outCloud (new pcl::PointCloud<PointOut>);
        //float red = pcl::generateColor (254, 0, 0);
        for (size_t i = 0; i < planes.size (); i++)
        {
          pcl::Plane3D<PointOut>& plane = planes[i];
          pcl::PointCloud<PointOut> tmpCloud = *plane.getInputCloud ();
          pcl::colorCloud (tmpCloud, pcl::color::getRandomColor (0.1, 2.9));
          *outCloud += tmpCloud;
        }
        return outCloud;
      }

      inline typename pcl::PointCloud<PointOut>::Ptr getRawPlanesCloud ()
      {
        typename pcl::PointCloud<PointOut>::Ptr outCloud (new pcl::PointCloud<PointOut>);
        //float red = pcl::generateColor (254, 0, 0);
        for (size_t i = 0; i < rawPlanes.size (); i++)
        {
          pcl::Plane3D<PointOut>& plane = rawPlanes[i];
          pcl::PointCloud<PointOut> tmpCloud = *plane.getInputCloud ();
          pcl::colorCloud (tmpCloud, pcl::color::getRandomColor (0.1, 2.9));
          *outCloud += tmpCloud;
        }
        return outCloud;
      }

      pcl::PolygonMesh::Ptr getMesh ()
      {
        return mesh;
      }

      Plane3DVector getPlanes ()
      {
        return planes;
      }

      Plane3DVector getWalls ()
      {
        return walls;
      }

      float getRotationangle ()
      {
        return angle;
      }
  };
}

#define PCL_INSTANTIATE_PlaneSegmentation(In,Out) template class PCL_EXPORTS pcl::PlaneSegmentation<In,Out>;

#endif /* PLANESEGMENTATION_H_ */
