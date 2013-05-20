/*
 * plane3d.h
 *
 *  Created on: Jun 22, 2012
 *      Author: elmasry
 *      This class utilisies the region growing algorithm by Dirk Holz
 *      Its a data structure for holding the detected planes
 */

#ifndef PLANE3D_H_
#define PLANE3D_H_

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include <pcl/common/transforms.h>
#include "pcl/common/edges_common.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include "pcl/common/math.h"
#include "pcl/common/point_common.h"

#include "pcl/types/boundingbox.h"
#include "pcl/common/pcl_commons.h"
#include "pcl/types/linesegment3d.h"
#include <pcl/registration/icp_dirk.h>
#include "pcl/registration/registration_dirk.h"

namespace pcl
{
  /**
   * PointT has to be of type pcl::PointXYZRGBNormal or an inheritance of it
   */
  template<typename PointT>
  class Plane3D : public pcl::PCLBase<PointT>
  {

    protected:
      size_t id;

      const static float ANGLE_THRESHHOLD = 15;
      static const float minStepLength = 0.2f;
      const static float minRiserHeight = 0.05f;
      const static float maxRiserHeight = 0.30;
      const static float minTreadDepth = 0.1f;
      const static float smallMinimalDepth = 0.05;
      //max distance between two lines to be considered lieing on the same plane
      const static float maxDistLinesOnSamePlane = 0.15;
      //max distance allowed in bbox for ex. max height deviation between top left and top right, or bottom left and bottom right
      const static float MAX_DIFF_BBOX_POINTS = 0.05;
      //max distance between two lines to consider it a slope to a step. used to reconstruct plane from 2 line segments
      const static float MAX_SLOPE_LENGTH_Z = 0.17;
      const static float MAX_SLOPE_LENGTH_X = 0.15;

      float length, lDepth, rDepth, height;
      //lines forming the plane model
      std::vector<LineSegment3D<PointT>, Eigen::aligned_allocator<PointT> > linesModel;

    public:
      using PCLBase<PointT>::input_;
      typedef boost::shared_ptr<Plane3D<PointT> > Ptr;

      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > PointsVector;

      Eigen::Vector3f center, normal;
      BoundingBox<PointT> bbox;
      std::vector<PointT, Eigen::aligned_allocator<PointT> > lastAddedPoints;
      std::vector<PointT, Eigen::aligned_allocator<PointT> > convexHull;

    protected:

      /**
       * finds neighboring points to a reference point. Distance comparison is done only on one axis, not euclidean
       */
      inline std::vector<PointT, Eigen::aligned_allocator<PointT> > findNeighborPointsOnAxis (
          const PointT& referencePoint, const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points, int axis,
          float distanceThreshold = 0.05)
      {
        std::vector<PointT, Eigen::aligned_allocator<PointT> > outPoints;
        for (size_t i = 0; i < points.size (); i++)
        {
          float dist;
          switch (axis)
          {
            case 0:
              dist = fabs (referencePoint.x - points[i].x);
              break;
            case 1:
              dist = fabs (referencePoint.y - points[i].y);
              break;
            case 2:
              dist = fabs (referencePoint.z - points[i].z);
              break;
            default:
              throw std::runtime_error ("pcl::findNeighborPointsOnAxis axis must be between 0-2");
          }
          if (dist < distanceThreshold)
            outPoints.push_back (points[i]);
        }
        return outPoints;
      }

      void setBBox (LineSegment3D<PointT> topLine, LineSegment3D<PointT> bottomLine)
      {
        bbox.topLeft = topLine.getLineModel  ()[0];
        bbox.topRight = topLine.getLineModel  ()[1];
        bbox.bottomLeft = bottomLine.getLineModel  ()[0];
        bbox.bottomRight = bottomLine.getLineModel  ()[1];
      }

      void stichCloud (typename pcl::PointCloud<PointT>::Ptr otherCloud)
      {
//        printf ("inputCloud.Size=%d and otherCloud.Size=%d\n", (int)input_->size (), (int)otherCloud->size ());
        typename pcl::PointCloud<PointT>::Ptr cloud_target (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud (*input_, *cloud_target);
        typename pcl::PointCloud<PointT>::Ptr cloud_source (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud (*otherCloud, *cloud_source);
        typename pcl::PointCloud<PointT>::Ptr outputCloud (new pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr cloud_aligned (new pcl::PointCloud<PointT>);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        pcl::CorrespondencesPtr correspondences_ptr (new pcl::Correspondences);
        icp4<PointT>(cloud_target, cloud_source, cloud_aligned, transform, correspondences_ptr, 1.0f);
        correspondences_ptr = findCorrespondances<PointT> (cloud_aligned, cloud_target);
//        correspondences_ptr = findCorrespondances<PointT> (cloud_source, cloud_target);
//        printf ("stichCloud() find correspondances.size=%d\n", (int) correspondences_ptr->size ());

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (cloud_source);
        pcl::PointIndices::Ptr indices = findCorrespondancesIndicesInSource (correspondences_ptr);
//        printf ("stichCloud: source.size=%d, indices found=%d\n", (int) cloud_source->size (),
//            (int) indices->indices.size ());
        if (indices->indices.size () > 0)
        {
          extract.setIndices (indices);
          extract.setNegative (true);
          extract.filter (*outputCloud);

          lastAddedPoints.clear ();
          lastAddedPoints.insert (lastAddedPoints.end (), outputCloud->begin (), outputCloud->end ());
//          printf ("stichCloud(): adding %d points to plane:%d\n", (int) outputCloud->size (), (int) id);
          outputCloud->insert (outputCloud->end (), input_->begin (), input_->end ());
          input_ = outputCloud;
        }
      }

      inline void calcBBox ()
      {
        convexHull.clear ();
        //convex hull algorithm doesn't work with small number of points
        if (input_->size () > 4)
        {
          //printf ("calculating ConvexHull. input.size=%d\n", input_->size ());
          try
          {
            convexHull = calcConvexHull<PointT> (input_);
          }
          catch (std::exception e)
          {
            if (convexHull.size () == 0)
            {
              PCL_ERROR( "pcl::calBBox. empty convex hull. num points input=%d\n", input_->size ());
              PointT p = initZeroPoint<PointT> ();
              bbox.topLeft = bbox.topRight = bbox.bottomLeft = bbox.bottomRight = p;
              return;
            }
          }

        }
        else
        {
          //if the number of points is too small, assign them all as a convex hull and find the maximas
//          convexHull.assign (input_->begin (), input_->end ());
          return;
        }

        typename PointCloud::Ptr tmpCloud (new PointCloud);
//        copyPointCloud (*input_, *tmpCloud);
        float distance_between_points = 0.01;
        if (convexHull.size () > 1)
        {
          for (size_t i = 0; i < convexHull.size () - 1; i++)
          {
            typename PointCloud::Ptr cloud = reconstructLineSegment (convexHull[i], convexHull[i + 1],
                distance_between_points);
            tmpCloud->insert (tmpCloud->end (), cloud->begin (), cloud->end ());
          }
          typename PointCloud::Ptr cloud = reconstructLineSegment (convexHull[convexHull.size () - 1], convexHull[0],
              distance_between_points);
          tmpCloud->insert (tmpCloud->end (), cloud->begin (), cloud->end ());
          //this->input_ = tmpCloud;
          convexHull.insert (convexHull.end (), tmpCloud->begin (), tmpCloud->end ());
        }
        PointsVector pointsYAxis = pcl::sortPoints (convexHull, 1);
        PointsVector leftPoints, rightPoints;
        //int clusterSize = 5;
//        size_t num_points = pointsYAxis.size () > clusterSize ? clusterSize : pointsYAxis.size ();
        //take 20% of the points
        float percent = 100;
        size_t num_points = pointsYAxis.size () * percent / 100;
        for (size_t i = 0; i < num_points; i++)
        {
          rightPoints.push_back (pointsYAxis[i]);
          leftPoints.push_back (pointsYAxis[pointsYAxis.size () - 1 - i]);
        }

        float maxDistBetweenPoints = 0.05;
        if (this->isRiser ())
        {
          //PointsVector pointsZAxisLeft = ::pcl::sortPoints (leftPoints, 2);
          PointsVector pointsZAxisLeft, pointsZAxisRight;
          int counter = 0;
          pointsZAxisLeft.push_back (leftPoints[0]);
          while (abs (leftPoints[0].y - leftPoints[++counter].y) < maxDistBetweenPoints)
          {
            pointsZAxisLeft.push_back (leftPoints[counter]);
          }
          pointsZAxisLeft = ::pcl::sortPoints (pointsZAxisLeft, 2);
          bbox.topLeft = pointsZAxisLeft[pointsZAxisLeft.size () - 1];
          bbox.bottomLeft = pointsZAxisLeft[0];

          counter = 0;
          pointsZAxisRight.push_back (rightPoints[0]);
          while (abs (rightPoints[0].y - rightPoints[++counter].y) < maxDistBetweenPoints)
          {
            pointsZAxisRight.push_back (rightPoints[counter]);
          }
          pointsZAxisRight = ::pcl::sortPoints (pointsZAxisRight, 2);
          bbox.topRight = pointsZAxisRight[pointsZAxisRight.size () - 1];
          bbox.bottomRight = pointsZAxisRight[0];
        }

        else if (this->isTread ())
        {

          PointsVector pointsXAxisLeft, pointsXAxisRight;
          int counter = 0;
          pointsXAxisLeft.push_back (leftPoints[0]);
          while (abs (leftPoints[0].y - leftPoints[++counter].y) < maxDistBetweenPoints)
          {
            pointsXAxisLeft.push_back (leftPoints[counter]);
          }
          pointsXAxisLeft = ::pcl::sortPoints (pointsXAxisLeft, 0);
          bbox.topLeft = pointsXAxisLeft[pointsXAxisLeft.size () - 1];
          bbox.bottomLeft = pointsXAxisLeft[0];

          counter = 0;
          pointsXAxisRight.push_back (rightPoints[0]);
          while (abs (rightPoints[0].y - rightPoints[++counter].y) < maxDistBetweenPoints)
          {
            pointsXAxisRight.push_back (rightPoints[counter]);
          }
          pointsXAxisRight = ::pcl::sortPoints (pointsXAxisRight, 0);
          bbox.topRight = pointsXAxisRight[pointsXAxisRight.size () - 1];
          bbox.bottomRight = pointsXAxisRight[0];
        }
      }

      inline void updateBBox (const BoundingBox<PointT>& bbox)
      {
        if (this->isRiser ())
        {
          if (this->bbox.topLeft.z > bbox.topLeft.z && this->bbox.topLeft.y > bbox.topLeft.y)
          {
            this->bbox.topLeft = bbox.topLeft;
          }
          if (this->bbox.bottomLeft.z < bbox.bottomLeft.z && this->bbox.bottomLeft.y > bbox.bottomLeft.y)
          {
            this->bbox.bottomLeft = bbox.bottomLeft;
          }

          if (this->bbox.topRight.z > bbox.topRight.z && this->bbox.topRight.y < bbox.topRight.y)
          {
            this->bbox.topRight = bbox.topRight;
          }
          if (this->bbox.bottomRight.z < bbox.bottomRight.z && this->bbox.bottomRight.y < bbox.bottomRight.y)
          {
            this->bbox.bottomRight = bbox.bottomRight;
          }
        }
        else if (this->isTread ())
        {
          if (this->bbox.topLeft.x > bbox.topLeft.x && this->bbox.topLeft.y > bbox.topLeft.y)
          {
            this->bbox.topLeft = bbox.topLeft;
          }
          if (this->bbox.bottomLeft.x < bbox.bottomLeft.x && this->bbox.bottomLeft.y > bbox.bottomLeft.y)
          {
            this->bbox.bottomLeft = bbox.bottomLeft;
          }

          if (this->bbox.topRight.x > bbox.topRight.x && this->bbox.topRight.y < bbox.topRight.y)
          {
            this->bbox.topRight = bbox.topRight;
          }
          if (this->bbox.bottomRight.x < bbox.bottomRight.x && this->bbox.bottomRight.y < bbox.bottomRight.y)
          {
            this->bbox.bottomRight = bbox.bottomRight;
          }
        }
      }

      void updateNormal (Eigen::Vector3f normal)
      {
        if ( (this->normal[0] > 0 && normal[0] < 0) || (this->normal[0] < 0 && normal[0] > 0))
          normal[0] *= -1;
        if ( (this->normal[1] > 0 && normal[1] < 0) || (this->normal[1] < 0 && normal[1] > 0))
          normal[1] *= -1;
        if ( (this->normal[2] > 0 && normal[2] < 0) || (this->normal[2] < 0 && normal[2] > 0))
          normal[2] *= -1;
        this->normal += normal;
        this->normal /= 2;
      }

      void updateCenter (Eigen::Vector3f center)
      {
        this->center += center;
        this->center /= 2;
      }

      void calculateCenter ()
      {
        center[0] = center[1] = center[2] = 0.0f;
        for (size_t i = 0; i < input_->size (); i++)
        {
          const PointT& p = input_->at (i);
          center += p.getVector3fMap ();
        }
        center /= input_->size ();
      }

      void calculateNormal ()
      {
        normal[0] = normal[1] = normal[2] = 0.0f;
        for (size_t i = 0; i < input_->size (); i++)
        {
          const PointT& p = input_->at (i);
          normal += p.getNormalVector3fMap ();
        }
        normal /= input_->size ();
      }
      void recalculateNormal ()
      {
        pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);

        pcl::FastMeshing<PointT, PointT> mesh_builder;
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
          std::cerr << "error estimating normal" << std::endl;
          return;
        }
        typename pcl::PointCloud<PointT>::Ptr cloud_mesh (new pcl::PointCloud<PointT> ());
        pcl::surface::computeNormalsAreaWeighted (*cloud_copy, mesh->polygons, *cloud_mesh);
        pcl::concatenateFields (*cloud_mesh, *cloud_copy, *cloud_mesh);

        bool colorize_wrt_normals = false;
        if (colorize_wrt_normals)
          pcl::surface::makeNormalColors (*cloud_mesh);
        pcl::toROSMsg (*cloud_mesh, mesh->cloud);

        bool smooth_mesh = true;
        if (smooth_mesh)
        {
          std::vector<std::vector<int> > vertex_list;
          pcl::getVertexVertexList (*mesh, vertex_list);

          // Bilateral Filtering!!!!!
          unsigned int filter_runs = 2;
          typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> (*cloud_mesh));
          for (unsigned int run = 0; run < filter_runs; ++run)
          {
            typename pcl::PointCloud<PointT>::Ptr cloud_temp (new pcl::PointCloud<PointT> ());
            pcl::surface::smoothBilateralNEW (*cloud_filtered, vertex_list, *cloud_temp);
            cloud_filtered.swap (cloud_temp);
          }

          for (size_t i = 0; i < cloud_mesh->points.size (); ++i)
          {
            cloud_filtered->points[i].rgb = cloud_mesh->points[i].rgb;
            cloud_filtered->points[i].id = cloud_mesh->points[i].id;
          }
          input_ = cloud_filtered;
        }

        calculateNormal ();
      }

    public:
      Plane3D ()
      {
        length = 0;
        lDepth = 0;
        rDepth = 0;
        height = 0;
        input_ = typename pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);

      }

      Plane3D (size_t id)
      {
        this->id = id;
        length = 0;
        lDepth = 0;
        rDepth = 0;
        height = 0;
        input_ = typename pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
      }

      Plane3D (const Plane3D<PointT>& plane3d) :
          PCLBase<PointT> ((PCLBase<PointT> ) plane3d)
      {
        Plane3D<PointT>::operator= (plane3d);
      }

      Plane3D<PointT>&
      operator= (const Plane3D<PointT>& plane3d)
      {
        this->id = plane3d.id;
        this->input_ = plane3d.input_;
        this->center = plane3d.center;
        this->normal = plane3d.normal;
        this->convexHull = plane3d.convexHull;
        this->bbox = plane3d.bbox;
        this->length = plane3d.length;
        this->lDepth = plane3d.lDepth;
        this->rDepth = plane3d.rDepth;
        this->height = plane3d.height;
        this->lastAddedPoints = plane3d.lastAddedPoints;
        return (*this);
      }

      bool operator< (const Plane3D<PointT>& plane) const
      {
        return (center[2] < plane.center[2]);
      }

      Plane3D<PointT>&
      operator+= (Plane3D<PointT>& plane)
      {

        size_t num_points_per_model = 4;

        //if this plane is plane model and input plane is a plane model
        if (this->input_->size () == 0 && plane.getCloud ()->size () == 0)
        {
          //if the two planes are only models
          updateCenter (plane.getCenter ());
          updateNormal (plane.getCenter ());
          updateBBox (plane.getBBox ());
          calculateDimensions ();
        }
        else if (this->input_->size () == 0 || plane.getCloud ()->size () == 0)
        {
          //if one plane is a model, the other is a real plane with points, then add the points together and recaclculate the bbox
//          printf("merging planes:(%d,%d)\n", this->id, plane.getId());
          if (plane.getCloud ()->size () > 0)
          {
            typename pcl::PointCloud<PointT>::Ptr cloud = plane.getCloud ();
            this->input_ = cloud;
            lastAddedPoints.clear ();
            lastAddedPoints.insert (lastAddedPoints.end (), cloud->begin (), cloud->end ());
//            lastAddedPoints = plane.getCloud()->points();
          }

          updateCenter (plane.getCenter ());
          updateNormal (plane.getCenter ());
          calcBBox ();
          calculateDimensions ();

        }
        else
        {
          size_t oldSize = input_->size ();
          stichCloud (plane.getCloud ());
          size_t newSize = input_->size ();
          if (newSize > oldSize)
          {
            updateCenter (plane.getCenter ());
//            updateNormal (plane.getCenter ());
//            recalculateNormal();
            calcBBox ();
            calculateDimensions ();
          }
        }
        return (*this);
      }

      /** \brief Provide a pointer to the input dataset
       * \param cloud the const boost shared pointer to a PointCloud message
       */
      virtual inline void setInputCloud (const PointCloudConstPtr &cloud, bool calculateCenterNormal = true,
          bool calculateBBox = true)
      {
        input_ = cloud;
        lastAddedPoints.clear ();
        typename pcl::PointCloud<PointT>::Ptr lastAddedCloud (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud (*input_, *lastAddedCloud);
        lastAddedPoints.insert (lastAddedPoints.end (), lastAddedCloud->begin (), lastAddedCloud->end ());

        if (calculateCenterNormal)
        {
          calculateCenter ();
          calculateNormal ();
        }

        if (calculateBBox)
        {
          calcBBox ();
          calculateDimensions ();
        }
      }

      inline void calculateDimensions ()
      {
        if (this->isTread ())
        {
          this->length = fabs (this->bbox.topRight.y - this->bbox.bottomLeft.y);
          this->lDepth = fabs (this->bbox.topLeft.x - this->bbox.bottomLeft.x);
          this->rDepth = fabs (this->bbox.topRight.x - this->bbox.bottomRight.x);
        }
        else if (this->isRiser ())
        {
          this->length = fabs (this->bbox.topRight.y - this->bbox.bottomLeft.y);
          this->height = fabs (this->bbox.bottomLeft.z - this->bbox.topRight.z);
        }
      }

      /**
       * creates a wall out of the 3DPlane
       */
      virtual inline void setModelWall (const LineSegment3D<PointT>& topLine, const LineSegment3D<PointT>& bottomLine)
      {
        typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        normal = Eigen::Vector3f (1, 0, 0);
        cloud->push_back (topLine.getLineModel  ()[0]);
        cloud->push_back (topLine.getLineModel  ()[1]);
        cloud->push_back (bottomLine.getLineModel  ()[0]);
        cloud->push_back (bottomLine.getLineModel  ()[1]);

        this->input_ = cloud;

        center = (topLine.getCenter () + bottomLine.getCenter ()) / 2;

        if (input_->size () <= 4)
        {
          //TODO I am willingly corrupting wall detection. I will repair it later
//          calcBBox (topLine, bottomLine);
//          calculateDimensions ();
        }

      }

      virtual inline void setPlaneModel (const PointT& topLeft, const PointT& topRight, const PointT& bottomLeft,
          const PointT& bottomRight)
      {
        LineSegment3D<PointT> topLine, bottomLine;
        topLine.setLineModel (topLeft, bottomLeft);
        bottomLine.setLineModel (bottomLeft, bottomRight);
        setPlaneModel (topLine, bottomLine);
      }

      /**
       * Builds a plane based on the given information, including all planes attributes
       */
      virtual inline void setPlaneModel (const LineSegment3D<PointT>& topLine, const LineSegment3D<PointT>& bottomLine)
      {
        typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

        LineSegment3D<PointT> line1 = topLine, line2 = bottomLine;

        float zDiff = fabs (topLine.getCenter ()[2] - bottomLine.getCenter ()[2]);
        float xDiff = fabs (topLine.getCenter ()[0] - bottomLine.getCenter ()[0]);
//        printf ("zDiff=%f, xDiff=%f, ", zDiff, xDiff);
        if (zDiff < maxDistLinesOnSamePlane && xDiff < maxDistLinesOnSamePlane) {
          if (zDiff < xDiff) {
            normal = Eigen::Vector3f (0, 0, 1);
          } else {
            normal = Eigen::Vector3f (1, 0, 0);
          }
        } else if (zDiff < maxDistLinesOnSamePlane)
        {
          normal = Eigen::Vector3f (0, 0, 1);
          //printf (", isTread");
        }else if (xDiff < maxDistLinesOnSamePlane)
        {
          //printf (", isRiser");
          normal = Eigen::Vector3f (1, 0, 0);
        }
        else
        {
          float xDiff1 = bottomLine.getCenter ()[0] - topLine.getCenter ()[0];
          float zDiff1 = topLine.getCenter ()[2] - bottomLine.getCenter ()[2];
          float xDiff2 = topLine.getCenter ()[0] - bottomLine.getCenter ()[0];
          float zDiff2 = -bottomLine.getCenter ()[2] - topLine.getCenter ()[2];

          //used to create a riser. make the two lines have the same depth, so that a riser is created
          if (xDiff1 > 0 && xDiff1 < MAX_SLOPE_LENGTH_X && zDiff1 > 0 && zDiff1 < MAX_SLOPE_LENGTH_Z)
          {
            PointT& p1 = line2.getLineModel  ()[0], p2 = line2.getLineModel  ()[1];
            p1.x = line1.getLineModel  ()[0].x;
            p2.x = line1.getLineModel  ()[1].x;
          }
          else if (xDiff2 > 0 && xDiff2 < MAX_SLOPE_LENGTH_X && zDiff2 > 0 && zDiff2 < MAX_SLOPE_LENGTH_Z)
          {
            PointT& p1 = line1.getLineModel  ()[0], p2 = line1.getLineModel  ()[1];
            p1.x = line2.getLineModel  ()[0].x;
            p2.x = line2.getLineModel  ()[1].x;
          }
        }
        //normal
        //    = (topLine.getLineModel  ()[1].getVector3fMap () - topLine.getLineModel  ()[0].getVector3fMap ()).cross (
        //                                                                                                   bottomLine.getLineModel  ()[1].getVector3fMap ()
        //                                                                                                       - bottomLine.getLineModel  ()[0].getVector3fMap ());
        //normal.normalize ();

//        cloud->push_back (line1.getLineModel  ()[0]);
//        cloud->push_back (line1.getLineModel  ()[1]);
//        cloud->push_back (line2.getLineModel  ()[0]);
//        cloud->push_back (line2.getLineModel  ()[1]);
//
//        this->input_ = cloud;

        center = (line1.getCenter () + line2.getCenter ()) / 2;
        setBBox (topLine, bottomLine);
        calculateDimensions ();

      }
      virtual inline void reset ()
      {
        center.Zero ();
        normal.Zero ();
      }

      inline Ptr makeShared ()
      {
        return Ptr (new Plane3D<PointT> (*this));
      }

      inline bool isStandardSize () const
      {
        if (this->isTread ())
        {
          if (length > minStepLength && (lDepth > smallMinimalDepth || rDepth > smallMinimalDepth))
            return true;
          return false;
        }
        else if (this->isRiser ())
        {
          if (length >= minStepLength && height >= minRiserHeight && height <= maxRiserHeight)
            return true;
          return false;
        }
        return false;
      }

      inline bool isMinimalSize () const
      {
        if (this->isTread ())
        {
          if (length > minStepLength && (lDepth > smallMinimalDepth || rDepth > smallMinimalDepth))
            return true;
          return false;
        }
        else if (this->isRiser ())
        {
          if (length >= minStepLength && height >= minRiserHeight && height <= maxRiserHeight)
            return true;
          return false;
        }
        return false;
      }

      inline void logPlane () const
      {
//        printf ("plane: %d", (int) id);
//        printf (", n=(%f,%f,%f)", normal[0], normal[1], normal[2]);
//        printf (", c=(%f,%f,%f)", center[0], center[1], center[2]);
//        printf (", angle to (1,0,0), (0,1,0). (0,0,1)=(%f,%f,%f)",
//            pcl::calcAngle3DDegree (normal, Eigen::Vector3f (1, 0, 0)),
//            pcl::calcAngle3DDegree (normal, Eigen::Vector3f (0, 1, 0)),
//            pcl::calcAngle3DDegree (normal, Eigen::Vector3f (0, 0, 1)));
        if (this->isTread ())
        {
          printf (", Tread l=%f, lDepth=%f, rDepth=%f", this->length, this->lDepth, this->rDepth);
        }
        else if (this->isRiser ())
        {
          printf (", Riser l=%f, h=%f", this->length, this->height);
        }
//        printf (", numPoints=%d", (int) this->getCloud ()->size ());
//        printf (", tl=(%f,%f,%f),tr=(%f,%f,%f),bl=(%f,%f,%f),br=(%f,%f,%f)", this->bbox.topLeft.x, this->bbox.topLeft.y,
//            this->bbox.topLeft.z, this->bbox.topRight.x, this->bbox.topRight.y, this->bbox.topRight.z,
//            this->bbox.bottomLeft.x, this->bbox.bottomLeft.y, this->bbox.bottomLeft.z, this->bbox.bottomRight.x,
//            this->bbox.bottomRight.y, this->bbox.bottomRight.z);
        printf ("\n");
      }

      size_t getId () const
      {
        return id;
      }

      void setId (size_t id)
      {
        this->id = id;
      }

      inline Eigen::Vector3f getNormal () const
      {
        return normal;
      }

      inline Eigen::Vector3f getCenter () const
      {
        return center;
      }

      typename PointCloud::Ptr getConvexHull () const
      {
        typename PointCloud::Ptr cloud (new PointCloud);
        cloud->insert (cloud->end (), convexHull.begin (), convexHull.end ());
        return cloud;
      }

      inline bool isHorizontal () const
      {
        return isHorizontal (Eigen::Vector3f (0, 0, 1));
      }

      inline bool isVertical () const
      {
        return isVertical (Eigen::Vector3f (0, 0, 1));
      }

      inline bool isHorizontal (Eigen::Vector3f refVector) const
      {
        float angle = pcl::getAngle (normal, refVector);
        if (angle < ANGLE_THRESHHOLD || fabs (180 - angle) < ANGLE_THRESHHOLD)
          return true;
        return false;
      }

      inline bool isVertical (Eigen::Vector3f refVector) const
      {
        float angle = pcl::getAngle (normal, refVector);
        if (fabs (90 - angle) < ANGLE_THRESHHOLD)
          return true;
        return false;
      }

      inline bool isTread () const
      {
//        if (isVertical (Eigen::Vector3f (1, 0, 0)) && isVertical (Eigen::Vector3f (0, 1, 0))
//            && isHorizontal (Eigen::Vector3f (0, 0, 1)))
//          return true;
//        return false;
        return isHorizontal ();
      }

      inline bool isRiser () const
      {
//        if (isHorizontal (Eigen::Vector3f (1, 0, 0)) && isVertical (Eigen::Vector3f (0, 1, 0))
//            && isVertical (Eigen::Vector3f (0, 0, 1)))
//          return true;
//        return false;
        return isVertical ();
      }

      virtual inline typename pcl::PointCloud<PointT>::Ptr getCloud () const
      {
        //return input_;
        typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        cloud->insert (cloud->end (), input_->begin (), input_->end ());
        return cloud;
      }

      virtual inline typename pcl::PointCloud<PointT>::ConstPtr getInputCloud () const
      {
        return input_;
      }

      BoundingBox<PointT> getBBox () const
      {
        return bbox;
      }

      void setLength (float length)
      {
        this->length = length;
      }

      float getLength () const
      {
        return length;
      }

      void setLDepth (float lDepth)
      {
        this->lDepth = lDepth;
      }

      float getLDEpth () const
      {
        return lDepth;
      }

      void setRDepth (float rDepth)
      {
        this->rDepth = rDepth;
      }

      float getRDepth () const
      {
        return rDepth;
      }

      void setHeight (float height)
      {
        this->height = height;
      }

      float getHeight () const
      {
        return height;
      }

      void setNormal (Eigen::Vector3f normal)
      {
        this->normal = normal;
      }

      void setCenter (Eigen::Vector3f center)
      {
        this->center = center;
      }

      void translate (PointT translation)
      {
        center += translation.getVector3fMap ();
        bbox.bottomLeft = sumPoints (bbox.bottomLeft, translation);
        bbox.topRight = sumPoints (bbox.topRight, translation);
        bbox.bottomRight = sumPoints (bbox.bottomRight, translation);
        bbox.topRight = sumPoints (bbox.topRight, translation);
      }

      void transform (Eigen::Matrix4f transformation)
      {
        //pointcloud
        typename pcl::PointCloud<PointT>::Ptr transCloud (new pcl::PointCloud<PointT>);
        transformPointCloud (*input_, *transCloud, transformation);
        this->input_ = transCloud;

        typename pcl::PointCloud<PointT>::Ptr lastAddedCloud (new pcl::PointCloud<PointT>);
        lastAddedCloud->insert (lastAddedCloud->end (), lastAddedPoints.begin (), lastAddedPoints.end ());
        transformPointCloud (*lastAddedCloud, *lastAddedCloud, transformation);
        lastAddedPoints.clear ();
        lastAddedPoints.insert (lastAddedPoints.end (), lastAddedCloud->begin (), lastAddedCloud->end ());

        //center
        Eigen::Transform<float, 3, Eigen::Affine> t (transformation);
        center = transformPoint (pcl::PointXYZ (center[0], center[1], center[2]), t).getVector3fMap ();
        //normal
//        normal = transformPoint (pcl::PointXYZ (normal[0], normal[1], normal[2]), t).getVector3fMap ();
        recalculateNormal ();
        //convexhull
//        for (size_t i = 0; i < convexHull.size (); i++)
//        {
//          convexHull[i] = transformPoint (convexHull[i], t);
//        }
        //bbox
//        bbox.bottomLeft = transformPoint (bbox.bottomLeft, t);
//        bbox.topRight = transformPoint (bbox.topRight, t);
//        bbox.bottomRight = transformPoint (bbox.bottomRight, t);
//        bbox.topRight = transformPoint (bbox.topRight, t);
        calcBBox ();
      }

      LineSegment3D<PointT> getTopLine () const
      {
        LineSegment3D<PointT> line;
        line.setLineModel (bbox.topLeft, bbox.topRight);
        return line;
      }

      LineSegment3D<PointT> getBottomLine () const
      {
        LineSegment3D<PointT> line;
        line.setLineModel (bbox.bottomLeft, bbox.bottomRight);
        return line;
      }

      float getHesseDistance () const
      {
        return center.dot (normal) * -1;
      }
  };
}

#define PCL_INSTANTIATE_Plane3D(T) template class PCL_EXPORTS pcl::Plane3D<T>;
#endif /* PLANE3D_H_ */
