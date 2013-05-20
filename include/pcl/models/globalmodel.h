/*
 * globalmodel.h
 *
 *  Created on: Oct 8, 2012
 *      Author: elmasry
 */

#ifndef GLOBALMODEL_H_
#define GLOBALMODEL_H_

#include <deque>

#include <boost/optional.hpp>
#include <pcl/visualization/registration_visualizer.h>

#include "pcl/common/point_common.h"
#include "pcl/common/pcl_commons.h";
#include "pcl/registration/icp_dirk.h"
#include "pcl/registration/registration_dirk.h"

#include "pcl/models/localmodel.h"
#include "pcl/models/modelsampler.h"
#include "pcl/io/mo_io.h"
#include "pcl/utils/pointcloud_utils.h"

namespace pcl
{
  template<typename PointT>
  class GlobalModel : public LocalModel<PointT>
  {

      //max distance tolerated between frames
      const static float MAX_DIST_BET_FRAMES = 0.15;
      const static size_t MINNUMMODELS = 5, MAXNUMMODELS = 10;
      //max distance allowed between steps to consider them a match
      const static float MAXDEVIATIONSTEPS = 0.15;

      template<typename Point>
      struct Match
      {
          size_t step1, step2;
          PointT translation;

          Match ()
          {
          }
          Match (const Match<PointT>& match)
          {
            Match<PointT>::operator= (match);
          }

          Match<PointT>&
          operator= (const Match<PointT>& match)
          {
            this->step1 = match.step1;
            this->step2 = match.step2;
            this->translation = match.translation;
            return (*this);
          }
      };

    protected:

      typedef Eigen::aligned_allocator<PointT> Alloc;
      typedef std::vector<Step<PointT>, Alloc> StepsVector;
      typedef std::vector<pcl::Plane3D<PointT>, Alloc> Plane3DVector;
      typedef std::deque<PointT, Alloc> PointsDequeue;
      typedef std::vector<PointT, Alloc> PointsVector;

      PointT translation;

      int numIterations;
      size_t stepsIndex, wallsIndex;

      PointsDequeue avgTranslation;
      Eigen::Matrix4f incrementalTransform;

      std::string cloudName;

      typename pcl::PointCloud<PointT>::Ptr lastGlobalModelCloud;

    public:
      std::deque<LocalModel<PointT> > localModels;

    protected:
      size_t findMaxStepsIndex ()
      {
        size_t maxIndex = 0;
        for (size_t i = 0; i < this->steps.size (); i++)
        {
          maxIndex = this->steps[i].getId () > maxIndex ? this->steps[i].getId () : maxIndex;
        }
        return maxIndex;
      }

      size_t findMaxWallsIndex ()
      {
        size_t maxIndex = 0;
        for (size_t i = 0; i < this->walls.size (); i++)
        {
          maxIndex = this->walls[i].getId () > maxIndex ? this->walls[i].getId () : maxIndex;
        }
        return maxIndex;
      }

      bool mergeLocalToGlobalModel (LocalModel<PointT>& localModel)
      {
        const StepsVector& localSteps = localModel.getSteps ();

        std::set<int> matchesSet;
        std::set<int>::iterator matchesIt;
        const float DIST_THRESHOLD_PLANES = 0.1;
        const float MAX_ANGLE_DEVIATION = 10.0f;
        const float DIST_THRESHOLD_BBOXSIDE = 0.3;

        for (size_t i = 0; i < localSteps.size (); i++)
        {
          Step<PointT> localStep = localSteps[i];
//          printf ("Inspecting step %d\n", localStep.getId ());
//          if ( (localStep.hasRiser () && !localStep.getRiser ().isRiser ())
//              || (localStep.hasTread () && !localStep.getTread ().isTread ()))
//          {
//            printf ("This step has invalid riser or tread.skip\n");
//            continue;
//          }

          std::map<float, size_t> distances = findNearestStepMatches ( (*this), localSteps[i]);
          std::map<float, size_t>::iterator distancesIT = distances.begin ();
          //float distance = 999;
          bool matchFound = false;
          typename StepsVector::iterator stepIt;
          if (distances.size () > 0 && distancesIT->first < MAXDEVIATIONSTEPS)
          {
//            printf ("step %d might be merged\n", localStep.getId ());
            for (stepIt = this->steps.begin (); stepIt != this->steps.end (); stepIt++)
            {
              //retrieving a pointer to the step in the global model
              if ( (*stepIt).getId () == distancesIT->second)
                break;
            }
            matchFound = true;
            if (stepIt != this->steps.end ())
            {
              if (stepIt->hasRiser () && localSteps[i].hasRiser ())
              {
                const Riser<PointT>& r1 = stepIt->getRiser ();
                const Riser<PointT>& r2 = localSteps[i].getRiser ();
                if (!r2.isRiser ())
                  continue;
                float angle = pcl::calcAngle3DDegree (r1.normal, r2.normal);

                if (/*pcl::euclideanDistance (r1.bbox.topLeft, r2.bbox.topLeft) > DIST_THRESHOLD_BBOXSIDE
                    || pcl::euclideanDistance (r1.bbox.topRight, r2.bbox.topRight) > DIST_THRESHOLD_BBOXSIDE
                    || pcl::euclideanDistance (r1.bbox.bottomLeft, r2.bbox.bottomLeft) > DIST_THRESHOLD_BBOXSIDE
                    || pcl::euclideanDistance (r1.bbox.bottomRight, r2.bbox.bottomRight) > DIST_THRESHOLD_BBOXSIDE
                    || fabs (r1.center[2] - r2.center[2]) > DIST_THRESHOLD_PLANES || angle > MAX_ANGLE_DEVIATION*/false)
                {
                  matchFound = false;
//                  printf ("step has bad riser\n");
//                  continue;
                }
              }

              if (stepIt->hasTread () && localSteps[i].hasTread ())
              {
                const Tread<PointT>& t1 = stepIt->getTread ();
                const Tread<PointT>& t2 = localSteps[i].getTread ();
                float angle = pcl::calcAngle3DDegree (t1.normal, t2.normal);
                if (/*pcl::euclideanDistance (t1.bbox.topLeft, t2.bbox.topLeft) > DIST_THRESHOLD_BBOXSIDE
                    || pcl::euclideanDistance (t1.bbox.topRight, t2.bbox.topRight) > DIST_THRESHOLD_BBOXSIDE
                    || pcl::euclideanDistance (t1.bbox.bottomLeft, t2.bbox.bottomLeft) > DIST_THRESHOLD_BBOXSIDE
                    || pcl::euclideanDistance (t1.bbox.bottomRight, t2.bbox.bottomRight) > DIST_THRESHOLD_BBOXSIDE
                    || fabs (t1.center[0] - t2.center[0]) > DIST_THRESHOLD_PLANES || angle > MAX_ANGLE_DEVIATION*/false)
                {
                  matchFound = false;
//                  printf ("step has bad tread\n");
//                  continue;
                }
              }
            }
            //distance = distancesIT->first;

            if (matchFound)
            {
//              printf ("match has been found\n");
              if (distances.size () > 0)
              {
                //a global step can be matched only once, also step index has to be > last step index, since we go from near step to far step

                //find the iterator to the next step to the local model step we want to insert
                for (stepIt = this->steps.begin (); stepIt != this->steps.end (); stepIt++)
                {
                  if ( (*stepIt).getId () == distancesIT->second)
                    break;
                }
              }

              //TODO add to matchesMap
              matchesIt = matchesSet.find ( (*stepIt).getId ());
              if (matchesIt != matchesSet.end ())
                continue;
              //            matchesSet.insert ( (*stepIt).getId ());
              //            printf ("merging steps: (%d,%d)\n", (*stepIt).getId (), localSteps[i].getId ());
              (*stepIt) += localSteps[i];
            }
          }

          else
          {
//            printf ("inserting new step: %d)\n", localSteps[i].getId ());
            if (distances.size () > 0)
            {
              //find the iterator to the next step to the local model step we want to insert
              for (stepIt = this->steps.begin (); stepIt != this->steps.end (); stepIt++)
              {
                if ( (*stepIt).getId () == distancesIT->second)
                  break;
              }
            }
            if (this->isLookingDownstairs ())
            {
              //TODO uncomment landing check
              //isLookingDownstairs
              if (localSteps[i].getCenter ()[0] > stepIt->getCenter ()[0]
                  && localSteps[i].getCenter ()[2] < stepIt->getCenter ()[2]
                  /*&& ! (stepIt->isLanding () && stepIt != this->steps.begin ()) */
                  )
              {
                //case new step is after current(closest) step in global model
                insertStep (++stepIt, localSteps[i]);
              }
              else if (localSteps[i].getCenter ()[0] <= stepIt->getCenter ()[0]
                  && localSteps[i].getCenter ()[2] >= stepIt->getCenter ()[2])
              {
                //case: new step is before current(closest) step in global model
                insertStep (stepIt, localSteps[i]);
              }
            }
            else if (this->isLookingUpstairs ())
            {
              const Step<PointT>& locS = localSteps[i];
              const float DIST_THRESHOLD_BORDERS = 0.1f;
              const float DIST_THRESHOLD_STEPS = 0.3;
//              const float DIST_THRESHOLD_STEPS =
              //isLooking upstairs as default
              if (locS.getCenter ()[0] > stepIt->getCenter ()[0] && locS.getCenter ()[2] > stepIt->getCenter ()[2]
              /*&& ! (stepIt->isLanding () && stepIt != this->steps.begin ())*/)
              {
                //new step is after the nearest step
                if (stepIt->hasTread () && locS.hasRiser ())
                {
                  Eigen::Vector3f c1 = (stepIt->getTread ().bbox.topLeft.getVector3fMap ()
                      + stepIt->getTread ().bbox.topRight.getVector3fMap ()) / 2;
                  Eigen::Vector3f c2 = (locS.getRiser ().bbox.bottomLeft.getVector3fMap ()
                      + locS.getRiser ().bbox.bottomRight.getVector3fMap ()) / 2;
                  if (fabs (c1[0] - c2[0]) > DIST_THRESHOLD_BORDERS || fabs (c1[2] - c2[2] > DIST_THRESHOLD_BORDERS))
                  {
                    continue;
                  }
                }
                else
                {
                  if (fabs (stepIt->center[0] - locS.center[0]) > DIST_THRESHOLD_STEPS
                      || fabs (stepIt->center[2] - locS.center[2]) > DIST_THRESHOLD_STEPS)
                  {
                    continue;
                  }
                }
                //case new step is after current(closest) step in global model
                insertStep (++stepIt, locS);
              }
              else if (locS.getCenter ()[0] <= stepIt->getCenter ()[0]
                  && locS.getCenter ()[2] <= stepIt->getCenter ()[2])
              {
                //case: new step is before current(closest) step in global model
                if (stepIt->hasRiser () && locS.hasTread ())
                {
                  Eigen::Vector3f c1 = (stepIt->getRiser ().bbox.bottomLeft.getVector3fMap ()
                      + stepIt->getRiser ().bbox.bottomRight.getVector3fMap ()) / 2;
                  Eigen::Vector3f c2 = (locS.getTread ().bbox.topLeft.getVector3fMap ()
                      + locS.getTread ().bbox.topRight.getVector3fMap ()) / 2;
                  if (fabs (c1[0] - c2[0]) > DIST_THRESHOLD_BORDERS || fabs (c1[2] - c2[2] > DIST_THRESHOLD_BORDERS))
                  {
                    continue;
                  }
                }
                else
                {
                  if (fabs (stepIt->center[0] - locS.center[0]) > DIST_THRESHOLD_STEPS
                      || fabs (stepIt->center[2] - locS.center[2]) > DIST_THRESHOLD_STEPS)
                  {
                    continue;
                  }
                }
                insertStep (stepIt, locS);
              }
            }
          }
        }
        std::sort (this->steps.begin (), this->steps.end ());
        //printf ("\n");
        return true;
      }

      /**
       * returns an index to the nearest match in the given local model compared to the given step from the other model
       */

      std::map<float, size_t> findNearestStepMatches (const LocalModel<PointT>& localModel, const Step<PointT>& step)
      {
        const StepsVector& steps = localModel.getSteps ();
        std::map<float, size_t> distances;
        for (size_t j = 0; j < steps.size (); j++)
        {
          const Step<PointT>& step2 = steps[j];
          int avgDeniminator = 0;
          float dist = 0;
          if (step.hasTread () && step2.hasTread ())
          {
            dist += fabs (step.getTread ().getCenter ()[2] - step2.getTread ().getCenter ()[2]);
            avgDeniminator += 1;
          }

          if (step.hasRiser () && step2.hasRiser ())
          {
            dist += fabs (step.getRiser ().getCenter ()[0] - step2.getRiser ().getCenter ()[0]);
            avgDeniminator += 1;
          }

          if (avgDeniminator == 0)
          {  //not both steps have treads and/or both have risers
            dist += euclideanDistance (PointXYZ (step.getCenter ()[0], step.getCenter ()[1], step.getCenter ()[2]),
                PointXYZ (step2.getCenter ()[0], step2.getCenter ()[1], step2.getCenter ()[2]));
          }
          else
          {
            dist /= avgDeniminator;
          }
          distances[dist] = step2.getId ();
        }

        //        printf (", for step:%d, pairs=", (int)step.getId());
        //        for (std::map<float, size_t>::iterator it = distances.begin (); it != distances.end (); it++)
        //        {
        //          printf ("(%f,%d),", it->first, (int)it->second);
        //        }
        return distances;
      }

      /**
       * Handles the model1 as the reference model and compare the model2 againest it
       */
      PointT findTranslationBetweenModels (const LocalModel<PointT>& model1, const LocalModel<PointT>& model2)
      {
        int matches = 0;
        PointT translation = pcl::initZeroPoint<PointT> ();
        const StepsVector& localSteps2 = model2.getSteps ();

        //<globalStepId, Match obj>
        std::map<size_t, Match<PointT> > matchesMap;
        //key=loclStepID, val=<distances, IDs of globalSteps>
        std::map<size_t, std::map<float, size_t> > localStepsMatches;

        //printf ("model1.steps=%d, model2.steps=%d\n", (int)model1.getSteps ().size (), (int)model2.getSteps ().size ());

        for (size_t i = 0; i < localSteps2.size (); i++)
        {
          //printf ("\n");
          //printf ("find match for model2: %d", (int)localSteps2[i].getId ());
          std::map<float, size_t> distances = findNearestStepMatches (model1, localSteps2[i]);
          localStepsMatches[localSteps2[i].getId ()] = distances;

          if (distances.size () == 0)
          {
            //printf (", no validstep");
            continue;
          }
          std::map<float, size_t>::iterator itDistances = distances.begin ();

          typename StepsVector::iterator stepsIt;
          for (stepsIt = model1.getSteps ().begin (); stepsIt != model1.getSteps ().end (); stepsIt++)
          {
            if (stepsIt->getId () == itDistances->second)
              break;
          }

          const Step<PointT>& step1 = (*stepsIt);
          //printf (", match=model1: %d, distance=%f", (int)step1.getId (), itDistances->first);
          //either they both have tread, and/or both have riser,
          if ( (step1.hasTread () && !localSteps2[i].hasTread ()) || (!step1.hasTread () && localSteps2[i].hasTread ())
              || (step1.hasRiser () && !localSteps2[i].hasRiser ())
              || (!step1.hasRiser () && localSteps2[i].hasRiser ()))
          {
            //printf (", not identical tread/riser");
            continue;
          }

          PointT tmpTranslation = vectorToPoint<PointT> (step1.getCenter () - localSteps2[i].getCenter ());
          typename std::map<size_t, Match<PointT> >::iterator it = matchesMap.find (step1.getId ());

          if (it == matchesMap.end ())
          {
            float dist = sqrt (
                pow (step1.getCenter ()[0] - localSteps2[i].getCenter ()[0], 2)
                    + pow (step1.getCenter ()[2] - localSteps2[i].getCenter ()[2], 2));
            //a match has been found
            if (dist < MAXDEVIATIONSTEPS)
            {
              matches++;
              Match<PointT> m;
              m.step1 = step1.getId ();
              m.step2 = localSteps2[i].getId ();
              m.translation = tmpTranslation;
              matchesMap[step1.getId ()] = m;
              //printf (", MATCH!!! distance=%f ,translation=(%f,%f,%f)", itDistances->first, tmpTranslation.x,
              //        tmpTranslation.y, tmpTranslation.z);
            }
            else
            {
              //printf(", distSteps is 2 large");
            }
          }
          else
          {
            //printf (", slot already used\n");
          }
        }

        printf ("\n");
        for (typename std::map<size_t, Match<PointT> >::iterator it = matchesMap.begin (); it != matchesMap.end ();
            it++)
        {
          //printf ("Find trans:match bet loc1:%d, loc:%d=(%f,%f,%f)\n", (int)it->second.step1, (int)it->second.step2,
          //        it->second.translation.x, it->second.translation.y, it->second.translation.z);
          translation = sumPoints (translation, it->second.translation);
        }

        return translation;
      }

      /**
       * iterates through all steps, and if a landing is found attached to a riser, it is
       * separated to its own step landing
       */
//      void separateLanding ()
//      {
//        typename StepsVector::iterator it;
//        for (it = this->steps.begin (); it != this->steps.end (); it++)
//        {
//          if (it->isLanding () && it->hasRiser ())
//          {
//            this->steps.erase (it + 1);
//            Step<PointT> step;
//            step.setRiser (it->getRiser ());
//            this->steps.push_back (step);
//            Step<PointT> landing;
//            landing.setTread (it->getTread ());
//            this->steps.push_back (landing);
//            ++it;
//          }
//        }
//      }
      void separateLanding ()
      {
        typename StepsVector::iterator it;
        StepsVector tmpSteps;
        for (it = this->steps.begin (); it != this->steps.end (); it++)
        {
          if (it->isLanding () && it->hasRiser ())
          {
            Step<PointT> step;
            step.setRiser (it->getRiser ());
            tmpSteps.push_back (step);
            Step<PointT> landing;
            landing.setTread (it->getTread ());
            tmpSteps.push_back (landing);
          }
          else
          {
            tmpSteps.push_back (*it);
          }
        }
        this->steps = tmpSteps;
      }

      void insertStep (typename StepsVector::iterator& it, const Step<PointT>& step)
      {
        //printf (", insert step");
        Step<PointT> s = step;
        s.setId (++stepsIndex);
        this->steps.insert (it, s);
      }

      void pushbackStep (const Step<PointT>& step)
      {
        //printf (", pushback step");
        Step<PointT> s = step;
        s.setId (++stepsIndex);
        this->steps.push_back (s);
      }

      /**
       * increments a localtranslation to the global translation.
       * localTranslation cannot affect global translation by more than maxPercent
       */
      void incrementGlobalTranslation (const PointT& localTranslation, float maxPercent = 30)
      {
        PointT tmpLocalTranslation = localTranslation;
        PointT tmpGlobal = ::pcl::sumPoints (translation, localTranslation);
        float xPercent, yPercent, zPercent;
        tmpLocalTranslation.x =
            (fabs (fabs (tmpGlobal.x) - fabs (translation.x)) / fabs (translation.x) * 100) > maxPercent ?
                tmpLocalTranslation.x : tmpLocalTranslation.x;

      }

      PointT calculateAveragePoint (const PointsDequeue& points)
      {
        PointT out = ::pcl::initZeroPoint<PointT> ();
        size_t numPoints = points.size ();
        for (size_t i = 0; i < points.size (); i++)
        {
          //if its a zero point, it shouldn't affect the avg
          if (::pcl::isZeroPoint (points[i]) && numPoints > 1)
          {
            numPoints--;
            continue;
          }
          out = sumPoints (out, points[i]);
        }
        out.x /= (int) points.size ();
        out.y /= (int) points.size ();
        out.z /= (int) points.size ();
        return out;
      }

      void extractCorrespondancesToClouds (typename pcl::PointCloud<PointT>::Ptr cloud_input_target,
          typename pcl::PointCloud<PointT>::Ptr cloud_input_source, pcl::CorrespondencesPtr correspondences_ptr,
          typename pcl::PointCloud<PointT>::Ptr cloud_output_target,
          typename pcl::PointCloud<PointT>::Ptr cloud_output_source)
      {
        for (size_t i = 0; i < correspondences_ptr->size (); i++)
        {
          cloud_output_source->push_back ( (*cloud_input_source)[ (*correspondences_ptr)[i].index_query]);
          cloud_output_target->push_back ( (*cloud_input_target)[ (*correspondences_ptr)[i].index_match]);
        }
      }

      void clearLastAddedPoints ()
      {
        for (size_t i = 0; i < this->steps.size (); i++)
        {
          Step<PointT> step = this->steps[i];
          if (step.hasRiser ())
          {
            Riser<PointT> riser = step.getRiser ();
            riser.lastAddedPoints.clear ();
            step.setRiser (riser);
          }
          if (step.hasTread ())
          {
            Tread<PointT> tread = step.getTread ();
            tread.lastAddedPoints.clear ();
            step.setTread (tread);
          }
          this->steps[i] = step;
        }
      }

      typename pcl::PointCloud<PointT>::Ptr getLastAddedPoints ()
      {
        typename pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);
        for (size_t i = 0; i < this->steps.size (); i++)
        {
          Step<PointT> step = this->steps[i];
          if (step.hasRiser ())
          {
            Riser<PointT> riser = step.getRiser ();
            PointsVector points = step.getRiser ().lastAddedPoints;
            output->insert (output->end (), points.begin (), points.end ());
          }
          if (step.hasTread ())
          {
            Tread<PointT> tread = step.getTread ();
            PointsVector points = step.getTread ().lastAddedPoints;
            output->insert (output->end (), points.begin (), points.end ());
          }
        }
        return output;
      }

    public:

      void reset ()
      {
        translation = pcl::initZeroPoint<PointT> ();
        numIterations = 0;
        this->steps.clear ();
        this->walls.clear ();
        stepsIndex = 0;
      }
      GlobalModel ()
      {
        reset ();
        incrementalTransform = Eigen::Matrix4f::Identity ();
      }

      void addLocalModel (LocalModel<PointT> localModel)
      {
        printf ("numiterations=%d\n", numIterations++);
        clearLastAddedPoints ();
        if (localModel.getSteps ().size () == 0)
          return;  //nothing to do, just continue
//        printf ("localModel before incremental transform:\n");
//        localModel.logSteps ();

        if (false)
        {
          ModelSampler<PointT> localmodelSampler;
          localmodelSampler.setModel (localModel);

//          char f[50];
//          sprintf (f, "localmodel_nontransformed_%i.pcd", numIterations);
//          ::pcl::io::saveModel<PointT> (std::string (f), localModel);
          std::vector<typename pcl::PointCloud<PointT>::Ptr> clouds_source;
          int pointsPerLine = 40;
          float regularity = 0.005;
//          clouds_source.push_back (localmodelSampler.getSampledConcaveEdges (regularity));
//          clouds_source.push_back (localmodelSampler.getSampledConvexEdges (regularity));
          clouds_source.push_back (localmodelSampler.getSampledLeftTreadEdges (pointsPerLine));
          clouds_source.push_back (localmodelSampler.getSampledRightTreadEdges (pointsPerLine));
          clouds_source.push_back (localmodelSampler.getSampledLeftRiserEdges (pointsPerLine));
          clouds_source.push_back (localmodelSampler.getSampledRightRiserEdges (pointsPerLine));
          clouds_source.push_back (localmodelSampler.getSampledBottomTreadEdges (regularity));
          clouds_source.push_back (localmodelSampler.getSampledTopTreadEdges (regularity));
          clouds_source.push_back (localmodelSampler.getSampledTopRiserEdges (regularity));
          clouds_source.push_back (localmodelSampler.getSampledBottomRiserEdges (regularity));

          typename PointCloud<PointT>::Ptr tmp_out1 (new PointCloud<PointT>);
          copyPointCloud (*localModel.getModelCloud (), *tmp_out1);
          float white = pcl::generateColor (253, 253, 253);
          float black1 = pcl::generateColor (30, 30, 30);
          float red = pcl::generateColor (253, 0, 0);
          float green = pcl::generateColor (0, 253, 0);
          pcl::colorCloud ( (*tmp_out1), black1);
          for (size_t i = 0; i < clouds_source.size (); ++i)
          {
            pcl::colorCloud ( (*clouds_source[i]), red);
            tmp_out1->insert (tmp_out1->end (), clouds_source[i]->begin (), clouds_source[i]->end ());
          }
//          if (tmp_out1->size () > 0)
//          {
//            char f[50];
//            sprintf (f, "featurecloud_nontranssource%d.pcd", numIterations);
//            pcl::io::savePCDFileBinary (string (f), *tmp_out1);
//          }
        }

        if (!incrementalTransform.isIdentity ())
        {
          localModel.transform (incrementalTransform);
          printf ("localModel after incremental transform:\n");
          localModel.logSteps ();
        }

        char f[50];
        sprintf (f, "localmodel_transformed_%d.pcd", numIterations);
        pcl::io::savePCDFileASCII (f, *localModel.getModelCloud ());

        if (false)
        {
          ModelSampler<PointT> localmodelSampler;
          localmodelSampler.setModel (localModel);

//          char f[50];
//          sprintf (f, "localmodel_transformed_%i.pcd", numIterations);
//          ::pcl::io::saveModel<PointT> (std::string (f), localModel);
          std::vector<typename pcl::PointCloud<PointT>::Ptr> clouds_source;
          int pointsPerLine = 40;
          float regularity = 0.005;
          clouds_source.push_back (localmodelSampler.getSampledConcaveEdges (regularity));
          clouds_source.push_back (localmodelSampler.getSampledConvexEdges (regularity));
          clouds_source.push_back (localmodelSampler.getSampledLeftTreadEdges (pointsPerLine));
          clouds_source.push_back (localmodelSampler.getSampledRightTreadEdges (pointsPerLine));
          clouds_source.push_back (localmodelSampler.getSampledLeftRiserEdges (pointsPerLine));
          clouds_source.push_back (localmodelSampler.getSampledRightRiserEdges (pointsPerLine));
          clouds_source.push_back (localmodelSampler.getSampledBottomTreadEdges (regularity));
          clouds_source.push_back (localmodelSampler.getSampledTopTreadEdges (regularity));
          clouds_source.push_back (localmodelSampler.getSampledTopRiserEdges (regularity));
          clouds_source.push_back (localmodelSampler.getSampledBottomRiserEdges (regularity));

//          clouds_source.push_back (localmodelSampler.getTreadsConvexHulls ());
//          clouds_source.push_back (localmodelSampler.getRisersConvexHulls ());

          typename PointCloud<PointT>::Ptr tmp_out1 (new PointCloud<PointT>);
          copyPointCloud (*localModel.getModelCloud (), *tmp_out1);
          float white = pcl::generateColor (253, 253, 253);
          float black1 = pcl::generateColor (30, 30, 30);
          float red = pcl::generateColor (253, 0, 0);
          float green = pcl::generateColor (0, 253, 0);
          pcl::colorCloud ( (*tmp_out1), black1);
          for (size_t i = 0; i < clouds_source.size (); ++i)
          {
            pcl::colorCloud ( (*clouds_source[i]), green);
            tmp_out1->insert (tmp_out1->end (), clouds_source[i]->begin (), clouds_source[i]->end ());
          }
//          if (tmp_out1->size () > 0)
//          {
//            char f[50];
//            sprintf (f, "featurecloud_transsource%d.pcd", numIterations);
//            pcl::io::savePCDFileBinary (string (f), *tmp_out1);
//          }
        }

        //printf ("localmodel.steps=%d\n", (int)tmpLocal.getSteps ().size ());
        if (this->steps.size () == 0 && localModel.getSteps ().size () > 0)
        {
          this->steps = localModel.getSteps ();
          this->walls = localModel.getWalls ();
          stepsIndex = findMaxStepsIndex () + 1;
          wallsIndex = findMaxWallsIndex () + 1;
        }
        else if (this->steps.size () > 0 && localModel.getSteps ().size () > 0)
        {
          ModelSampler<PointT> localmodelSampler, globalmodelSampler;
          localmodelSampler.setModel (localModel);
          globalmodelSampler.setModel (*this);

//          char f[50];
//          sprintf (f, "localmodel_%i.pcd", numIterations);
//          ::pcl::io::saveModel (std::string (f), localModel);
          std::vector<typename pcl::PointCloud<PointT>::Ptr> clouds_source, clouds_targets;
          int pointsPerLine = 40;
          float regularity = 0.005;
//          printf ("before adding local model\n");
//          clouds_targets.push_back (globalmodelSampler.getSampledConcaveEdges (regularity));
//          clouds_targets.push_back (globalmodelSampler.getSampledConvexEdges (regularity));
//          clouds_targets.push_back (globalmodelSampler.getSampledLeftTreadEdges (pointsPerLine));
//          clouds_targets.push_back (globalmodelSampler.getSampledRightTreadEdges (pointsPerLine));
//          clouds_targets.push_back (globalmodelSampler.getSampledLeftRiserEdges (pointsPerLine));
//          clouds_targets.push_back (globalmodelSampler.getSampledRightRiserEdges (pointsPerLine));
//          clouds_targets.push_back (globalmodelSampler.getSampledTopTreadEdges (regularity));
//          clouds_targets.push_back (globalmodelSampler.getSampledBottomTreadEdges (regularity));
//          clouds_targets.push_back (globalmodelSampler.getSampledTopRiserEdges (regularity));
//          clouds_targets.push_back (globalmodelSampler.getSampledBottomRiserEdges (regularity));
////          printf ("sampling globalmodel is done\n");
//          clouds_source.push_back (localmodelSampler.getSampledConcaveEdges (regularity));
//          clouds_source.push_back (localmodelSampler.getSampledConvexEdges (regularity));
//          clouds_source.push_back (localmodelSampler.getSampledLeftTreadEdges (pointsPerLine));
//          clouds_source.push_back (localmodelSampler.getSampledRightTreadEdges (pointsPerLine));
//          clouds_source.push_back (localmodelSampler.getSampledLeftRiserEdges (pointsPerLine));
//          clouds_source.push_back (localmodelSampler.getSampledRightRiserEdges (pointsPerLine));
//          clouds_source.push_back (localmodelSampler.getSampledTopTreadEdges (regularity));
//          clouds_source.push_back (localmodelSampler.getSampledBottomTreadEdges (regularity));
//          clouds_source.push_back (localmodelSampler.getSampledTopRiserEdges (regularity));
//          clouds_source.push_back (localmodelSampler.getSampledBottomRiserEdges (regularity));

//          clouds_targets.push_back (globalmodelSampler.getTreadsPoints ());
//          clouds_targets.push_back (globalmodelSampler.getRisersPoints ());
//          clouds_source.push_back (localmodelSampler.getTreadsPoints ());
//          clouds_source.push_back (localmodelSampler.getRisersPoints ());

          clouds_targets.push_back (globalmodelSampler.getTreadsConvexHulls ());
          clouds_targets.push_back (globalmodelSampler.getRisersConvexHulls ());
          clouds_source.push_back (localmodelSampler.getTreadsConvexHulls ());
          clouds_source.push_back (localmodelSampler.getRisersConvexHulls ());

//          printf ("after adding local model\n");
          int numClouds = (int) clouds_targets.size ();
          typename PointCloud<PointT>::Ptr tmp_out1 (new PointCloud<PointT>), tmp_out2 (new PointCloud<PointT>);
          typename PointCloud<PointT>::Ptr c1 = this->getModelCloud ();
          typename PointCloud<PointT>::Ptr c2 = localModel.getModelCloud ();
          copyPointCloud (*c1, *tmp_out1);
          copyPointCloud (*c2, *tmp_out2);
          float white = pcl::generateColor (253, 253, 253);
          float red = pcl::generateColor (253, 0, 0);
          float green = pcl::generateColor (0, 253, 0);
          pcl::colorCloud ( (*tmp_out1), white);
          pcl::colorCloud ( (*tmp_out2), white);
          for (size_t i = 0; i < numClouds; ++i)
          {
            pcl::colorCloud ( (*clouds_targets[i]), red);
            pcl::colorCloud ( (*clouds_source[i]), green);
            tmp_out1->insert (tmp_out1->end (), clouds_targets[i]->begin (), clouds_targets[i]->end ());
            tmp_out2->insert (tmp_out2->end (), clouds_source[i]->begin (), clouds_source[i]->end ());
          }
//          if (tmp_out1->size () > 0 && tmp_out2->size () > 0)
//          {
//            char f[50];
//            sprintf (f, "featurecloud_target%d.pcd", numIterations);
//            pcl::io::savePCDFileBinary (string (f), *tmp_out1);
//            sprintf (f, "featurecloud_source%d.pcd", numIterations);
//            pcl::io::savePCDFileBinary (string (f), *tmp_out2);
//          }

//          printf ("numClouds=%d\n", numClouds);
          typename PointCloud<PointT>::Ptr cloud_source_ptr (new PointCloud<PointT>), cloud_target_ptr (
              new PointCloud<PointT>), cloud_aligned_source (new PointCloud<PointT>);
          Eigen::Matrix4f localTransform = Eigen::Matrix4f::Identity ();
          const float ICP_MAX_POINT_DISTANCE = 1.0f;
          for (size_t i = 0; i < numClouds; ++i)
          {
//            printf ("cloudSource.size=%d, cloudTarget.size=%d. Going to downsample and filter\n",
//                (int) clouds_targets[i]->size (), (int) clouds_source[i]->size ());
//            bool downsampling = true;
//            if (downsampling)
//            {
//              // Create the filtering object
//              pcl::VoxelGrid<PointT> sor;
//              sor.setLeafSize (0.01f, 0.01f, 0.01f);
//
//              typename pcl::PointCloud<PointT>::Ptr cloud_temp (new pcl::PointCloud<PointT>);
//
//              sor.setInputCloud (clouds_targets[i]);
//              sor.filter (*cloud_temp);
//              clouds_targets[i].swap (cloud_temp);
//
//              sor.setInputCloud (clouds_source[i]);
//              sor.filter (*cloud_temp);
//              clouds_source[i].swap (cloud_temp);
//            }
//            std::vector<int> indices;
//            pcl::removeNaNFromPointCloud (*clouds_targets[i], *clouds_targets[i], indices);
//            pcl::removeNaNFromPointCloud (*clouds_source[i], *clouds_source[i], indices);
//            printf ("cloudSource.size=%d, cloudTarget.size=%d. after downsample and filter\n",
//                (int) clouds_source[i]->size (), (int) clouds_targets[i]->size ());

//            //doing an icp on each type of edges, then add the correspondance result into two point clouds
//            char f1[50], f2[50];
//            sprintf (f1, "target_%i.pcd", i);
//            if (clouds_targets[i]->size () > 0)
//              pcl::io::savePCDFileASCII (std::string (f1), * (clouds_targets[i]));
//            sprintf (f2, "source_%i.pcd", i);
//            if (clouds_source[i]->size () > 0)
//              pcl::io::savePCDFileASCII (std::string (f2), * (clouds_source[i]));
//            printf("Going to save cloud source\n");
//            if (clouds_source[i]->size () > 0)
//            {
//              char f[50];
////              printf ("using cloudName=%s\n", this->cloudName.c_str ());
////              sprintf ("%s_feature_%d.pcd", cloudName.c_str (), (int) i);
//              sprintf ("feature_%d.pcd", (int) i);
//              pcl::io::savePCDFileBinary (f, * (clouds_source[i]));
//            }
//            printf ("After saving cloud.source\n");
//            localModel.featureClouds.push_back (clouds_source[i]);
//            printf ("pushing back feature cloud\n");
            pcl::CorrespondencesPtr correspondences_ptr (new pcl::Correspondences);
            std::vector<int> indices_rejected;
//            printf ("Going to calculate icp. loop=%d\n", (int) i);
            icp4<PointT> (clouds_targets[i], clouds_source[i], cloud_aligned_source, localTransform,
                correspondences_ptr, ICP_MAX_POINT_DISTANCE);
//            printf ("icp calculated\n");
//            printf ("num correspondances=%d\n", (int) correspondences_ptr->size ());
            extractCorrespondancesToClouds (clouds_targets[i], clouds_source[i], correspondences_ptr, cloud_target_ptr,
                cloud_source_ptr);
          }

          //TODO localModels has to be later deleted, otherwise it will fill the memory
          localModels.push_back (localModel);
          if (localModels.size () > 3)
          {
            localModels.pop_front ();
          }

//          printf ("Doing the final icp\n");
          //doing an icp on the result of the four types of edges altogether
          if (cloud_target_ptr->size () > 0 && cloud_source_ptr->size () > 0)
          {
//            printf ("Doing icp is possible\n");
            pcl::CorrespondencesPtr correspondences_ptr (new pcl::Correspondences);
//            printf ("doing final icp. source.size=%d, target.size=%d\n", (int) cloud_target_ptr->size (),
//                (int) cloud_source_ptr->size ());
            icp4<PointT> (cloud_target_ptr, cloud_source_ptr, cloud_aligned_source, localTransform, correspondences_ptr,
                ICP_MAX_POINT_DISTANCE);
            icp5<PointT> (cloud_target_ptr, cloud_source_ptr, correspondences_ptr);


//            printf ("final icp num_correspondances=%d\n", (int) correspondences_ptr->size ());
//            std::cout << "localtransform= " << localTransform << std::endl;

            if (correspondences_ptr->size () > 0)
            {

              localModel.transform (localTransform);
              char f[50];
              sprintf (f, "localmodel_localtransform_%d.pcd", numIterations);
              pcl::io::savePCDFileASCII (f, *localModel.getModelCloud ());
//              printf ("localModel after avg transform:\n");
//              localModel.logSteps ();
              mergeLocalToGlobalModel (localModel);
              incrementalTransform = localTransform * incrementalTransform;
            }
          }
//          std::cout << "incrementalTransform= " << incrementalTransform << std::endl;
        }

//        Eigen::Vector4f translation = incrementalTransform.col (3);
//        incrementalTransform = Eigen::Matrix4f::Identity ();
//        incrementalTransform.col (3) = translation;
//        std::cout << "matrix =" << incrementalTransform << std::endl;
        ///DEBUG///////
//        printf ("saving global models pcds\n");
        char f1[50], f2[50], f3[50];
        float red = pcl::generateColor (255, 0, 0);
        typename pcl::PointCloud<PointT>::Ptr globalmodelCloud = this->getModelCloud ();
        sprintf (f1, "globalmodel_%d.pcd", numIterations);
        pcl::io::savePCDFileASCII (f1, *globalmodelCloud);
        float grey = pcl::generateColor (200, 200, 200);
        pcl::colorCloud (*globalmodelCloud, grey);
        sprintf (f2, "globalmodel_grey_%d.pcd", numIterations);
        pcl::io::savePCDFileASCII (f2, *globalmodelCloud);
        typename pcl::PointCloud<PointT>::Ptr lastAddedCloud = getLastAddedPoints ();
        pcl::colorCloud (*lastAddedCloud, red);
        pcl::concatePointClouds (*lastAddedCloud, *globalmodelCloud);
        sprintf (f3, "globalmodel_lastadded_%d.pcd", numIterations);
        pcl::io::savePCDFileASCII (f3, *globalmodelCloud);
        //// END DEBUG /////

        separateLanding ();
        printf ("globalModel\n");
        this->logSteps ();
      }

      void setCloudName (std::string cloudName)
      {
        this->cloudName = cloudName;
        printf ("called setCloudName=%s\n", this->cloudName.c_str ());
      }
  };
}

#define PCL_INSTANTIATE_GlobalModel(T) template class PCL_EXPORTS pcl::GlobalModel<T>;

#endif /* GLOBALMODEL_H_ */
