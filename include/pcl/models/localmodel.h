/*
 * localmode.h
 *
 *  Created on: Aug 16, 2012
 *      Author: elmasry
 */

#ifndef LOCALMODE_H_
#define LOCALMODE_H_

#include <vector>
#include <Eigen/Dense>

#include "pcl/utils/pointcloud_utils.h"
#include "pcl/types/plane3d_utils.h"

#include "pcl/types/step.h"

namespace pcl
{
  template<typename PointT>
  class LocalModel
  {
    protected:

      typedef Eigen::aligned_allocator<PointT> Alloc;
      typedef std::vector<Step<PointT>, Alloc> StepsVector;
      typedef std::vector<pcl::LineSegment3D<PointT>, Alloc> LineSegment3DVector;
      typedef std::vector<pcl::Plane3D<PointT>, Alloc> Plane3DVector;

      Plane3DVector walls;
    public:

      StepsVector steps;
      typedef boost::shared_ptr<LocalModel<PointT> > Ptr;
      typedef boost::shared_ptr<LocalModel<const PointT> > ConstPtr;
      //featureClouds like concave edges, convex edges, convexhulls etc..
      std::vector<typename PointCloud<PointT>::Ptr> featureClouds;

      LocalModel ()
      {
      }

      LocalModel (const LocalModel<PointT>& model)
      {
        LocalModel<PointT>::operator= (model);

      }

      LocalModel<PointT>&
      operator= (const LocalModel<PointT>& model)
      {
        this->steps = model.steps;
        this->walls = model.walls;
        return (*this);
      }

      void addStep (const Step<PointT>& step)
      {
        steps.push_back (step);
      }

      void setSteps (const StepsVector& steps)
      {
        this->steps = steps;
      }

      void reverseSteps ()
      {
        std::reverse (steps.begin (), steps.end ());
      }

      /**
       * get the detected steps as colored cloud, where each step has a different color
       */
      typename pcl::PointCloud<PointT>::Ptr getModelCloud ()
      {
        typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        for (size_t i = 0; i < steps.size (); i++)
        {
          Step<PointT>& step = steps[i];

          float color = pcl::color::getRandomColor (1.0f, 2.4f);
          if (step.hasRiser ())
          {
            typename pcl::PointCloud<PointT>::Ptr tmpCloud = step.getRiser ().getCloud ();
            if (tmpCloud->size () == 0)
            {
              tmpCloud = ::pcl::utils::sampleBoundingBox<PointT> (step.getRiser ());
            }
            pcl::colorCloud<PointT> (tmpCloud, color);
            pcl::concatePointClouds (*tmpCloud, *cloud);
          }
          if (step.hasTread ())
          {
            typename pcl::PointCloud<PointT>::Ptr tmpCloud = step.getTread ().getCloud ();
            if (tmpCloud->size () == 0)
            {
              tmpCloud = ::pcl::utils::sampleBoundingBox<PointT> (step.getTread ());
            }
            pcl::colorCloud<PointT> (tmpCloud, color);
            pcl::concatePointClouds (*tmpCloud, *cloud);
          }
        }
        return cloud;
      }

      void logSteps () const
      {
        printf ("numSteps=%d\n", (int) steps.size ());
        for (size_t i = 0; i < steps.size (); i++)
        {
          const Step<PointT>& step = steps[i];
          step.logStep ();
        }
      }

      void logStepsModified ()
      {
        int count = 0;
        size_t maxNumberSteps = 3;
        if (steps.size () >= maxNumberSteps)
        {
          printf ("%d\n", (int) maxNumberSteps);
        }
        else
        {
          printf ("%d\n", (int) steps.size ());
        }

        for (size_t i = 0; i < steps.size (); i++)
        {
          if (++count > 3)
            break;
          Step<PointT>& step = steps[i];
          if (step.hasTread ())
          {
            const Tread<PointT>& tread = step.getTread ();
            printf ("Tread %f %f %f\n", tread.getLength (), tread.getLDEpth (), tread.getRDepth ());
          }
          if (step.hasRiser ())
          {
            const Riser<PointT>& riser = step.getRiser ();
            printf ("Riser %f %f\n", riser.getLength (), riser.getHeight ());
          }
          printf ("\n");
        }
      }

      StepsVector getSteps () const
      {
        return steps;
      }

      bool isLookingUpstairs () const
      {
        if (steps.size () < 2)
          return false;
        Eigen::Vector3f firstCenter = steps.front ().getCenter (), lastCenter = steps.back ().getCenter ();
        return firstCenter[2] < lastCenter[2] && firstCenter[0] < lastCenter[0];
      }

      bool isLookingDownstairs () const
      {
        //if (steps.size () < 2) return false;
        //Eigen::Vector3f firstCenter = steps.front ().getCenter (), lastCenter = steps.back ().getCenter ();
        //return firstCenter[2] > lastCenter[2] && firstCenter[0] < lastCenter[0];
        return !isLookingUpstairs ();
      }

      void setWalls (const Plane3DVector& walls)
      {
        this->walls = walls;
      }

      void transform (Eigen::Matrix4f transformation)
      {
        for (size_t i = 0; i < steps.size (); i++)
        {
          Step<PointT>& step = steps[i];
          step.transform (transformation);
          steps[i] = step;
        }
      }

      Plane3DVector getWalls () const
      {
        return walls;
      }

      Eigen::Vector3f calculateCenterOfMass ()
      {
        Eigen::Vector3f center (0, 0, 0);
        for (size_t i = 0; i < steps.size (); i++)
        {
          center += steps[i].getCenter ();
        }
        center /= (int) steps.size ();
        return center;
      }

      LineSegment3DVector getModelLines ()
      {
        LineSegment3DVector lines;
        int counter = 0;
        for (size_t i = 0; i < steps.size (); i++)
        {
          const Step<PointT>& step = steps[i];
          LineSegment3D<PointT> upper, lower;
//          printf ("step.id=%d", (int) step.getId ());

          if (isLookingDownstairs ())
          {
            if (step.hasTread ())
            {
              const Tread<PointT>& tread = step.getTread ();
              if ((int) (i - 1) < 0 || (!steps[i - 1].hasRiser ()))
              {
                LineSegment3D<PointT> line1 (counter++, tread.getBBox ().bottomLeft, tread.getBBox ().bottomRight);
//                printf("bottomLine: bottomLeft=%f, bottomRight=%f\n", tread.getBBox ().bottomLeft, tread.getBBox ().bottomRight );
                line1.setContourType (ConcaveLine);
                lines.push_back (line1);
              }
              LineSegment3D<PointT> line2 (counter++, tread.getBBox ().topLeft, tread.getBBox ().topRight);
//              printf("topLine: topLeft=%f, topRight=%f\n", tread.getBBox ().topLeft, tread.getBBox ().topRight);
              line2.setContourType (ConvexLine);
              lines.push_back (line2);
            }

            if (step.hasRiser ())
            {
              const Riser<PointT>& riser = step.getRiser ();
              //if previous step tread wasnt used in finding lines
              if (!step.hasTread ())
              {
                LineSegment3D<PointT> line1 (counter++, riser.getBBox ().topLeft, riser.getBBox ().topRight);
//                printf("topLine: topLeft=%f, topRight=%f\n", riser.getBBox ().topLeft, riser.getBBox ().topRight);
                line1.setContourType (ConvexLine);
                lines.push_back (line1);
              }
              LineSegment3D<PointT> line2 (counter++, riser.getBBox ().bottomLeft, riser.getBBox ().bottomRight);
//              printf("bottomLine: bottomLeft=%f, bottomRight=%f\n", riser.getBBox ().bottomLeft, riser.getBBox ().bottomRight);
              line2.setContourType (ConcaveLine);
              lines.push_back (line2);
            }

          }
          else if (isLookingUpstairs ())
          {
            if (step.hasRiser ())
            {
              const Riser<PointT>& riser = step.getRiser ();
              //if previous step tread wasnt used in finding lines
              if ( (i >= 1 && (!steps[i - 1].hasTread ())) || (i == 0))
              {
                LineSegment3D<PointT> line1 (counter++, riser.getBBox ().bottomLeft, riser.getBBox ().bottomRight);
//                printf("bottomLine: bottomLeft=%f, bottomRight=%f\n", riser.getBBox ().bottomLeft, riser.getBBox ().bottomRight);
                line1.setContourType (ConcaveLine);
                lines.push_back (line1);
              }
              LineSegment3D<PointT> line2 (counter++, riser.getBBox ().topLeft, riser.getBBox ().topRight);
//              printf("topLine: topLeft=%f, topRight=%f\n", riser.getBBox ().topLeft, riser.getBBox ().topRight);
              line2.setContourType (ConvexLine);
              lines.push_back (line2);
            }

            if (step.hasTread ())
            {
              const Tread<PointT>& tread = step.getTread ();
              if (!step.hasRiser ())
              {
                LineSegment3D<PointT> line1 (counter++, tread.getBBox ().bottomLeft, tread.getBBox ().bottomRight);
//                printf("bottomLine: bottomLeft=%f, bottomRight=%f\n", tread.getBBox ().bottomLeft, tread.getBBox ().bottomRight);
                line1.setContourType (ConvexLine);
                lines.push_back (line1);
              }
              LineSegment3D<PointT> line2 (counter++, tread.getBBox ().topLeft, tread.getBBox ().topRight);
//              printf("topLine: topLeft=%f, topRight=%f\n", tread.getBBox ().topLeft, tread.getBBox ().topRight);
              line2.setContourType (ConcaveLine);
              lines.push_back (line2);
            }
          }
        }
        return lines;
      }

      typename pcl::PointCloud<PointT>::Ptr getBoundingBoxesCloud ()
      {
        typename pcl::PointCloud<PointT>::Ptr outCloud (new pcl::PointCloud<PointT>);

        for (size_t i = 0; i < steps.size (); i++)
        {
          const Step<PointT>& step = steps[i];
          if (step.hasRiser ())
          {
            pcl::concatePointClouds (*::pcl::utils::sampleBoundingBox<PointT> (step.getRiser ()), *outCloud);
          }

          if (step.hasTread ())
          {
            pcl::concatePointClouds (*pcl::utils::sampleBoundingBox<PointT> (step.getTread ()), *outCloud);
          }
        }
        return outCloud;
      }
  };
}

#define PCL_INSTANTIATE_LocalModel(T) template class PCL_EXPORTS pcl::LocalModel<T>;

#endif /* LOCALMODE_H_ */
