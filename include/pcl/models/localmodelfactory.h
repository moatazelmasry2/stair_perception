/*
 * localmodelfactory.h
 *
 *  Created on: Aug 21, 2012
 *      Author: elmasry
 */

#ifndef LOCALMODELFACTORY_H_
#define LOCALMODELFACTORY_H_

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>

#include "pcl/types/plane3d.h"
#include "pcl/types/step.h"
#include "pcl/common/color.h"
#include "pcl/utils/pointcloud_utils.h"
#include "pcl/types/plane3d_utils.h"
#include "pcl/types/tread.h"
#include "pcl/types/riser.h"
#include "pcl/models/localmodel.h"
#include "pcl/histogram/planehistogram.h"

namespace pcl
{
  template<typename PointT>
  class LocalModelFactory
  {
      static const int compIndex = 2;
      //camera coordinate system values
      static const float maxYDist = 0.8;
      static const float maxZDist = 0.4;
      static const float maxXDist = 0.4;
      static const float minStepLength = 0.5;
      static const float minStepDepth = 0.1;
      static const float minStepHeight = 0.1;
      static const float xDifferenceThreshold = 0.8;

      static const float xDeviationBetStepsThreshold = 0.4;
      static const float zDeviationBetStepsThreshold = 0.3;

      typedef Eigen::aligned_allocator<PointT> Alloc;
      typedef std::vector<Plane3D<PointT>, Alloc> Plane3DVector;
      typedef std::vector<Step<PointT>, Alloc> StepsVector;

      Plane3DVector planes;
      Plane3DVector walls;

    protected:

      int planeCounter;

      StepsVector createSteps (Plane3DVector& planes, bool useMinimalThreshold = false)
      {
        StepsVector steps;
//        printf("\n\n\n");
//        printf ("creating steps. numPlanes=%d\n", planes.size ());
        float negativeTolerance = -0.05;
        for (size_t i = 0; i < planes.size (); i++)
        {
          bool added = false;
          Plane3D<PointT>& plane = planes[i];
//          plane.logPlane ();

          if (!useMinimalThreshold && !plane.isStandardSize ())
          {
//            printf ("plane:%d dod not pass standard size test: continue\n", (int) plane.getId ());
            continue;
          }
          else if (useMinimalThreshold && !plane.isMinimalSize ())
          {
//            printf ("plane:%d dod not pass minimum size test: continue\n", (int) plane.getId ());
            continue;
          }

          if (plane.isRiser ())
          {
            typename Riser<PointT>::Ptr riser = boost::static_pointer_cast<Riser<PointT> > (plane.makeShared ());

            for (size_t j = 0; j < steps.size (); j++)
            {
              Step<PointT>& step = steps[j];
              if (!step.hasRiser () && !step.isLanding ())
              {
                const Tread<PointT>& tread = step.getTread ();
                float xDist = tread.getCenter ()[0] - plane.getCenter ()[0];
                float zDist = tread.getCenter ()[2] - plane.getCenter ()[2];
                bool overlaps = plane.getBBox ().bottomLeft.y > tread.getBBox ().topRight.y
                    || plane.getBBox ().bottomRight.y < tread.getBBox ().topLeft.y;

                if (xDist < maxXDist && xDist > negativeTolerance && zDist < maxZDist && zDist > negativeTolerance
                    && overlaps)
                {
                  added = true;
                  step.setRiser (*riser);
                  break;
                }
              }
            }
            if (!added)
            {
              Step<PointT> step ((int) i);

              step.setRiser (*riser);
              steps.push_back (step);
            }
          }

          else if (plane.isTread ())
          {
            typename Tread<PointT>::Ptr tread = boost::static_pointer_cast<Tread<PointT> > (plane.makeShared ());
            if (!tread->isLanding ())
            {
              for (size_t j = 0; j < steps.size (); j++)
              {
                Step<PointT>& step = steps[j];
                if (!step.hasTread ())
                {
                  const Plane3D<PointT>& riser = step.getRiser ();
                  float xDist = plane.getCenter ()[0] - riser.getCenter ()[0];
                  float zDist = plane.getCenter ()[2] - riser.getCenter ()[2];
                  bool overlaps = riser.getBBox ().bottomLeft.y > plane.getBBox ().topRight.y
                      || riser.getBBox ().bottomRight.y < plane.getBBox ().topLeft.y;

                  if (xDist < maxZDist && xDist > negativeTolerance && zDist < maxZDist && zDist > negativeTolerance
                      && overlaps)
                  {
                    added = true;
                    step.setTread (*tread);
                    break;
                  }
                }
              }
            }

            if (!added)
            {
              Step<PointT> step ((int) i);
              step.setTread (*tread);
              steps.push_back (step);
            }

          }
        }

//        printf("log steps after step create\n");
//        logSteps(steps);
        return steps;
      }

      Plane3DVector removeFarPlanes (const Plane3DVector& planes, float xDifferenceThreshold) const
      {
        Eigen::Vector3f avgCenter = pcl::utils::calcAvgCenters (planes);
        Plane3DVector tmpPlanes;

        for (size_t i = 0; i < planes.size (); i++)
        {
          if (fabs (planes[i].getCenter ()[1] - avgCenter[1]) < xDifferenceThreshold)
          {
            tmpPlanes.push_back (planes[i]);
          }
        }
        return tmpPlanes;
      }

      /**
       * Takes in a vector of ordered steps, builds a logical staircase out of them
       */
      /**
       * \throws exception if number of inputsteps = 0
       */
      StepsVector buildStairCase (StepsVector steps)
      {
        if (steps.size () == 0)
          throw runtime_error ("buildStairCase() number of steps=0");
        StepsVector tmpSteps, outVector;
        std::vector<StepsVector> tmpStaircases;

//        printf ("logging steps before building staircase. numsteps=%d\n", (int) steps.size ());

        for (size_t i = 0; i < steps.size (); i++)
        {
          Step<PointT>& step = steps[i];
//          step.logStep ();
          if (tmpStaircases.size () == 0)
          {
            StepsVector stepsVec;
            stepsVec.push_back (step);
            tmpStaircases.push_back (stepsVec);
//            printf ("creating the first staircase\n");
          }
          else if (tmpStaircases.size () == 1)
          {
            tmpStaircases[0].push_back (step);
          }
          else
          {
            for (size_t j = 0; j < tmpStaircases.size (); j++)
            {
              StepsVector& stepsVec = tmpStaircases[j];
              if (stepsVec.size () == 0)
                continue;
              Step<PointT>& lastStep = stepsVec.back ();

              float curStepLeftY = 0, curStepRightY = 0, lastStepLeftY = 0, lastStepRightY = 0;
              float xDeviationBetPlanes = 999, zDeviationBetPlanes = 999;
              if (lastStep.hasTread () && step.hasRiser ())
              {
                lastStepLeftY = lastStep.getTread ().getBBox ().bottomLeft.y;
                lastStepRightY = lastStep.getTread ().getBBox ().topRight.y;
                curStepLeftY = step.getRiser ().getBBox ().bottomLeft.y;
                curStepRightY = step.getRiser ().getBBox ().topRight.y;
                xDeviationBetPlanes = lastStep.getTread ().getBBox ().topRight.x
                    - step.getRiser ().getBBox ().bottomRight.x;
                zDeviationBetPlanes = lastStep.getTread ().getBBox ().topRight.z
                    - step.getRiser ().getBBox ().bottomRight.z;
              }
              else if (lastStep.hasTread () && step.hasTread ())
              {
                lastStepLeftY = lastStep.getTread ().getBBox ().bottomLeft.y;
                lastStepRightY = lastStep.getTread ().getBBox ().topRight.y;
                curStepLeftY = step.getTread ().getBBox ().bottomLeft.y;
                curStepRightY = step.getTread ().getBBox ().topRight.y;
                xDeviationBetPlanes = lastStep.getTread ().getBBox ().topRight.x
                    - step.getTread ().getBBox ().bottomRight.x;
                zDeviationBetPlanes = lastStep.getTread ().getBBox ().topRight.z
                    - step.getTread ().getBBox ().bottomRight.z;
              }
              else if (lastStep.hasRiser () && step.hasTread ())
              {
                lastStepLeftY = lastStep.getRiser ().getBBox ().bottomLeft.y;
                lastStepRightY = lastStep.getRiser ().getBBox ().topRight.y;
                curStepLeftY = step.getTread ().getBBox ().bottomLeft.y;
                curStepRightY = step.getTread ().getBBox ().topRight.y;
                xDeviationBetPlanes = lastStep.getRiser ().getBBox ().topRight.x
                    - step.getRiser ().getBBox ().bottomRight.x;
                zDeviationBetPlanes = lastStep.getRiser ().getBBox ().topRight.z
                    - step.getRiser ().getBBox ().bottomRight.z;
              }
              else if (lastStep.hasRiser () && step.hasRiser ())
              {
                lastStepLeftY = lastStep.getRiser ().getBBox ().bottomLeft.y;
                lastStepRightY = lastStep.getRiser ().getBBox ().topRight.y;
                curStepLeftY = step.getRiser ().getBBox ().bottomLeft.y;
                curStepRightY = step.getRiser ().getBBox ().topRight.y;
                xDeviationBetPlanes = lastStep.getRiser ().getBBox ().topRight.x
                    - step.getRiser ().getBBox ().bottomRight.x;
                zDeviationBetPlanes = lastStep.getRiser ().getBBox ().topRight.z
                    - step.getRiser ().getBBox ().bottomRight.z;
              }
              if (curStepLeftY > lastStepRightY && curStepRightY < lastStepLeftY && xDeviationBetPlanes > 0
                  && xDeviationBetPlanes <= xDeviationBetStepsThreshold && zDeviationBetPlanes > 0
                  && zDeviationBetPlanes <= zDeviationBetStepsThreshold)
              {
                printf ("Adding plane to stair to staircase: %d\n", (int) i);
                stepsVec.push_back (step);
                break;
              }

              //if this step doesnt fit to any staircase, then create a new staircase for it.
              if (j == tmpStaircases.size () - 1)
              {
//                printf ("this plane does not fit. create staircase num: %d\n", tmpStaircases.size () + 1);
                StepsVector stepsVec;
                stepsVec.push_back (step);
                tmpStaircases.push_back (stepsVec);
              }
            }
          }
        }

        //find the staircase with the largest number of steps
        if (tmpStaircases.size () == 0)
          throw runtime_error ("buildStairCase() num staircases=0");
        if (tmpStaircases.size () == 1)
          return tmpStaircases[0];
        size_t index = 0;
        size_t maxStairs = tmpStaircases[0].size ();
        for (size_t i = 1; i < tmpStaircases.size (); i++)
        {
          if (tmpStaircases[i].size () > maxStairs)
          {
            index = i;
            maxStairs = tmpStaircases[i].size ();
          }
        }
//        printf ("the winner staircase is: %d\n", (int) index);
        return tmpStaircases[index];
      }

      void addMissingPlanes (LocalModel<PointT>& model)
      {
        StepsVector steps = model.getSteps ();
        for (size_t i = 0; i < steps.size () - 1; i++)
        {
          Step<PointT>& step = steps[i];
          if (!step.hasRiser () && !step.isLanding ())
          {
            Step<PointT>& succ = steps[i + 1];
            if (succ.hasTread ())
            {
              Riser<PointT> riser (planeCounter++);
              LineSegment3D<PointT> topLine = step.getTread ().getTopLine ();
              LineSegment3D<PointT> bottomLine = succ.getTread ().getBottomLine ();
              std::vector<PointT, Eigen::aligned_allocator<PointT> > model = bottomLine.getLineModel ();
              model[0].x = topLine.getLineModel ()[0].x;
              model[1].x = topLine.getLineModel ()[1].x;
              bottomLine.setLineModel (model);
              riser.setPlaneModel (topLine, bottomLine);
              step.setRiser (riser);

              Tread<PointT> succTread = succ.getTread ();
              succTread.bbox.bottomRight = bottomLine.getLineModel ()[1];
              succTread.bbox.bottomLeft = bottomLine.getLineModel ()[0];
              succTread.calculateDimensions ();
              steps[i + 1].setTread (succTread);
            }
          }

          else if (!step.hasTread ())
          {
            if (i + 1 < steps.size ())
            {
              Step<PointT>& succ = steps[i + 1];
              if (succ.hasRiser ())
              {
                Tread<PointT> tread (planeCounter++);
                tread.setPlaneModel (succ.getRiser ().getTopLine (), step.getRiser ().getBottomLine ());
                step.setTread (tread);
              }
            }
          }
        }
        model.steps = steps;
      }

      LocalModel<PointT> removeGaps (LocalModel<PointT> model)
      {
        const float MAX_GAP_ALLOWED = 0.01;
        const float RELAX_FACTOR = 0.6;
        for (size_t i = 0; i < model.steps.size (); i++)
        {
          Step<PointT>& step = model.steps[i];
          if (step.hasTread () && step.hasRiser ())
          {
            Riser<PointT> riser = step.getRiser ();
            Tread<PointT> tread = step.getTread ();

            if (model.isLookingUpstairs ())
            {
              //clsing the gap between riser and tread in a single step
              Eigen::Vector3f c1 = (riser.bbox.topLeft.getVector3fMap () + riser.bbox.topRight.getVector3fMap ()) / 2;
              Eigen::Vector3f c2 = (tread.bbox.bottomLeft.getVector3fMap () + tread.bbox.bottomRight.getVector3fMap ())
                  / 2;
              if (fabs (c1[2] - c2[2]) > MAX_GAP_ALLOWED || fabs (c1[0] - c2[0]) > MAX_GAP_ALLOWED)
              {
                Eigen::Vector3f translation ( (c1[0] - c2[0]) * RELAX_FACTOR, 0, (c1[2] - c2[2]) * RELAX_FACTOR);
                Eigen::Matrix4f m = Eigen::Affine3f (Eigen::Translation3f (translation)).matrix ();
                tread.transform (m);
                step.setTread (tread);
                model.steps[i] = step;
                for (size_t j = i + 1; j < model.steps.size (); j++)
                {
                  Step<PointT>& step2 = model.steps[j];
                  step2.transform (m);
                  model.steps[j] = step2;
                }
              }

              //Now closing the gap between a tread in one step and a riser in the next step
              //if step has tread and is not last step, and next step has riser
              if (step.hasTread () && i < model.steps.size () - 1 && model.steps[i + 1].hasRiser ())
              {
                riser = model.steps[i + 1].getRiser ();
                c1 = (tread.bbox.topLeft.getVector3fMap () + tread.bbox.topRight.getVector3fMap ()) / 2;
                c2 = (riser.bbox.bottomLeft.getVector3fMap () + riser.bbox.bottomRight.getVector3fMap ()) / 2;

                if (fabs (c1[2] - c2[2]) > MAX_GAP_ALLOWED || fabs (c1[0] - c2[0]) > MAX_GAP_ALLOWED)
                {
                  Eigen::Vector3f translation ( (c1[0] - c2[0]) * RELAX_FACTOR, 0, (c1[2] - c2[2]) * RELAX_FACTOR);
                  Eigen::Matrix4f m = Eigen::Affine3f (Eigen::Translation3f (translation)).matrix ();
                  for (size_t j = i + 1; j < model.steps.size (); j++)
                  {
                    Step<PointT>& step2 = model.steps[j];
                    step2.transform (m);
                    model.steps[j] = step2;
                  }
                }

              }
            }
          }
        }
        return model;
      }

      LocalModel<PointT> flipNormals (LocalModel<PointT> model)
      {

        Eigen::Vector3f treadDefault (1, 1, -1), riserDefault (-1, 1, 1);

        for (size_t i = 0; i < model.steps.size (); i++)
        {

          Step<PointT>& step = model.steps[i];
          if (step.hasTread ())
          {
            Tread<PointT> tread = step.getTread ();

            if ( (tread.normal[0] > 0 && treadDefault[0] < 0) || (tread.normal[0] < 0 && treadDefault[0] > 0))
            {
              tread.normal[0] *= -1;
            }
            if ( (tread.normal[1] > 0 && treadDefault[1] < 0) || (tread.normal[1] < 0 && treadDefault[1] > 0))
            {
              tread.normal[1] *= -1;
            }

            if ( (tread.normal[2] > 0 && treadDefault[2] < 0) || (tread.normal[2] < 0 && treadDefault[2] > 0))
            {
              tread.normal[2] *= -1;
            }
            typename pcl::PointCloud<PointT>::Ptr cloud = tread.getCloud ();
            for (size_t i = 0; i < cloud->size (); i++)
            {
              PointT p = (*cloud)[i];
              if ( (p.normal_x > 0 && treadDefault[0] < 0) || (p.normal_x < 0 && treadDefault[0] > 0))
              {
                p.normal_x *= -1;
              }
              if ( (p.normal_y > 0 && treadDefault[1] < 0) || (p.normal_y < 0 && treadDefault[1] > 0))
              {
                p.normal_y *= -1;
              }
              if ( (p.normal_z > 0 && treadDefault[2] < 0) || (p.normal_z < 0 && treadDefault[2] > 0))
              {
                p.normal_z *= -1;
              }
              (*cloud)[i] = p;
            }
            tread.input_ = cloud;
            step.setTread (tread);
          }

          if (step.hasRiser ())
          {
            Riser<PointT> riser = step.getRiser ();
            if ( (riser.normal[0] > 0 && riserDefault[0] < 0) || (riser.normal[0] < 0 && riserDefault[0] > 0))
            {
              riser.normal[0] *= -1;
            }
            if ( (riser.normal[1] > 0 && riserDefault[1] < 0) || (riser.normal[1] < 0 && riserDefault[1] > 0))
            {
              riser.normal[1] *= -1;
            }

            if ( (riser.normal[2] > 0 && riserDefault[2] < 0) || (riser.normal[2] < 0 && riserDefault[2] > 0))
            {
              riser.normal[2] *= -1;
            }
            typename pcl::PointCloud<PointT>::Ptr cloud = riser.getCloud ();
            for (size_t i = 0; i < cloud->size (); i++)
            {
              PointT p = (*cloud)[i];
              if ( (p.normal_x > 0 && riserDefault[0] < 0) || (p.normal_x < 0 && riserDefault[0] > 0))
              {
                p.normal_x *= -1;
              }
              if ( (p.normal_y > 0 && riserDefault[1] < 0) || (p.normal_y < 0 && riserDefault[1] > 0))
              {
                p.normal_y *= -1;
              }
              if ( (p.normal_z > 0 && riserDefault[2] < 0) || (p.normal_z < 0 && riserDefault[2] > 0))
              {
                p.normal_z *= -1;
              }
              (*cloud)[i] = p;
            }
            riser.input_ = cloud;
            step.setRiser (riser);
          }
          model.steps[i] = step;
        }
        return model;
      }

      LocalModel<PointT> placementAligment (LocalModel<PointT> model)
      {
        for (size_t i = 0; i < model.steps.size (); i++)
        {
          Step<PointT>& step = model.steps[i];

          if (step.hasTread ())
          {
            Tread<PointT> tread = step.getTread ();
            typename pcl::PointCloud<PointT>::Ptr cloud = tread.getCloud ();
            for (size_t j = 0; j < cloud->size (); j++)
            {
              PointT& p = cloud->at (j);
              p.z = tread.center[2];
            }
            tread.input_ = cloud;
            step.setTread (tread);
            model.steps[i] = step;
          }
        }

        return model;
      }
    public:

      LocalModelFactory ()
      {
        planeCounter = 0;
      }

      void logSteps (const StepsVector& steps) const
      {
        printf ("numSteps=%d\n", (int) steps.size ());
        for (size_t i = 0; i < steps.size (); i++)
        {
          const Step<PointT>& step = steps[i];
          step.logStep ();
        }
      }

      void logPlanes ()
      {
        printf ("num planes=%d\n", (int) planes.size ());
        for (size_t i = 0; i < planes.size (); i++)
        {
          planes[i].logPlane ();

        }
      }

      inline void setPlanes (const Plane3DVector& planes)
      {
        this->planes = planes;
      }

      void addPlanes (Plane3DVector planes)
      {
        for (typename Plane3DVector::iterator it = planes.begin (); it != planes.end (); it++)
        {
          planeCounter++;
//          it->setId (planeCounter++);
          this->planes.push_back (*it);
        }
        //::pcl::utils::logPlanes(this->planes);
      }

      void mergePlanes ()
      {
        pcl::PlaneHistogram<pcl::Plane3D<PointT>, PointT> horHist (2), vertHist (0);
        for (size_t i = 0; i < planes.size (); i++)
        {
          Plane3D<PointT>& plane = planes[i];
          if (plane.isTread ())
          {
            horHist.addPlane (plane);
          }
          else if (plane.isRiser ())
          {
            vertHist.addPlane (plane);
          }
        }
        float threshold = 0.04, threshold2 = 0.1;
        horHist.setStepsize (threshold);
        vertHist.setStepsize (threshold);
        horHist.maxDistanceThreshold = threshold2;
        vertHist.maxDistanceThreshold = threshold2;
        horHist.compute ();
        vertHist.compute ();
        std::vector<Plane3D<PointT>, Eigen::aligned_allocator<Plane3D<PointT> > > hPlanes = horHist.getOutplanes (),
            vPlanes = vertHist.getOutplanes ();
        //int oldNumPlanes = planes.size (), newNumPlanes;
        planes.clear ();
        planes.insert (planes.end (), hPlanes.begin (), hPlanes.end ());
        planes.insert (planes.end (), vPlanes.begin (), vPlanes.end ());
        //::pcl::utils::logPlanes(planes);
        //newNumPlanes = planes.size ();
        //printf ("numMerges=%d\n", oldNumPlanes - newNumPlanes);
        //logPlanes (this->planes);
      }

      LocalModel<PointT> createLocalModel (bool useMinimalThreshold = false)
      {
//        logPlanes();
//        Plane3DVector outPlanes = removeFarPlanes (planes, xDifferenceThreshold);
        StepsVector steps = createSteps (planes, useMinimalThreshold);
        std::sort (steps.begin (), steps.end ());
        //logSteps (steps);

        LocalModel<PointT> model;
        if (steps.size () == 0)
          return model;
        steps = buildStairCase (steps);
        model.setSteps (steps);
//        model.setWalls (walls);
//        addMissingPlanes (model);
//        model = removeGaps (model);
//        model = flipNormals (model);

//        model = placementAligment(model);
        //if (model.isLookingDownstairs ())
        //{
        //  model.reverseSteps ();
        //}
        return model;
      }

      void addWalls (const Plane3DVector& walls)
      {
        this->walls.insert (this->walls.end (), walls.begin (), walls.end ());
      }

      void mergeWalls ()
      {
        if (walls.size () < 2)
          return;
        pcl::PlaneHistogram<pcl::Plane3D<PointT>, PointT> hist (1);
        float threshold = 0.04, threshold2 = 0.1;
        hist.setStepsize (threshold);
        hist.maxDistanceThreshold = threshold2;
        std::vector<Plane3D<PointT>, Eigen::aligned_allocator<Plane3D<PointT> > > walls2;
        walls2.assign (walls.begin (), walls.end ());
        //::pcl::utils::logPlanes (walls);
        //printf("logging histogram for walls\n");
        hist.addPlanes (walls2);
        hist.compute ();
        walls2 = hist.getOutplanes ();
        walls.clear ();
        walls.assign (walls2.begin (), walls2.end ());
      }
  };
}

#define PCL_INSTANTIATE_LocalModelFactory(T) template class PCL_EXPORTS pcl::LocalModelFactory<T>;

#endif /* LOCALMODELFACTORY_H_ */
