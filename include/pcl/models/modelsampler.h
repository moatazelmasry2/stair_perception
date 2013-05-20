/*
 * LocalModelSampler.h
 *
 *  Extracts samples from the LocalModel, for example, concave and convex edges, left and right edges, etc...
 *  Created on: Jan 13, 2013
 *      Author: elmasry
 */

#ifndef MODELSAMPLER_H_
#define MODELSAMPLER_H_

//#include "pcl/common/edges_common.h"
#include "pcl/models/localmodel.h"

namespace pcl
{
  template<typename PointT>
  class ModelSampler
  {
    private:

    protected:

      LocalModel<PointT> localModel;

      typedef Eigen::aligned_allocator<PointT> Alloc;
      typedef std::vector<Step<PointT>, Alloc> StepsVector;
      typedef std::vector<pcl::LineSegment3D<PointT>, Alloc> LineSegment3DVector;

      typename PointCloud<PointT>::Ptr sampleLines (const LineSegment3DVector& lines, int numSampledPoints)
      {
        typename PointCloud<PointT>::Ptr outCloud (new PointCloud<PointT>);

//        printf("sampleing %d lines\n", lines.size());
        for (size_t i = 0; i < lines.size (); i++)
        {
          const LineSegment3D<PointT>& line = lines[i];
          std::vector<PointT, Alloc> model = line.getLineModel  ();
          if (model.size () == 2)
          {
            typename PointCloud<PointT>::Ptr lineCloud = reconstructLineSegment (model[0], model[1], numSampledPoints);
            pcl::concatePointClouds (*lineCloud, *outCloud);
          }
          else
          {
            PCL_ERROR("LocalModelSampler:getConvexEdges number of points in lineModel should be 2\n");
          }

        }
        return outCloud;
      }

      typename PointCloud<PointT>::Ptr sampleLines (const LineSegment3DVector& lines, float regularity)
      {
        typename PointCloud<PointT>::Ptr outCloud (new PointCloud<PointT>);

        //        printf("sampleing %d lines\n", lines.size());
        for (size_t i = 0; i < lines.size (); i++)
        {
          const LineSegment3D<PointT>& line = lines[i];
          std::vector<PointT, Alloc> model = line.getLineModel  ();
          if (model.size () == 2)
          {
            typename PointCloud<PointT>::Ptr lineCloud = reconstructLineSegment (model[0], model[1], regularity);
            pcl::concatePointClouds (*lineCloud, *outCloud);
          }
          else
          {
            PCL_ERROR("LocalModelSampler:getConvexEdges number of points in lineModel should be 2\n");
          }

        }
        return outCloud;
      }

      LineSegment3DVector getConvexEdges ()
      {
        const LineSegment3DVector& lines = localModel.getModelLines();
        LineSegment3DVector convexLines;
        for (size_t i = 0; i < lines.size (); i++)
        {
          LineSegment3D<PointT> line = lines[i];
          if (ConvexLine == line.getContourType ())
          {
            convexLines.push_back (line);
          }
        }
        return convexLines;
      }

      LineSegment3DVector getConcaveEdges ()
      {
        const LineSegment3DVector& lines = localModel.getModelLines ();
        LineSegment3DVector concaveLines;
        for (size_t i = 0; i < lines.size (); i++)
        {
          LineSegment3D<PointT> line = lines[i];
          if (ConcaveLine == line.getContourType ())
          {
            concaveLines.push_back (line);
          }
        }
        return concaveLines;
      }

    public:

      inline void setModel (const LocalModel<PointT>& localModel)
      {
        this->localModel = localModel;
      }

      typename PointCloud<PointT>::Ptr getSampledConvexEdges (int numSampledPoints)
      {
        return sampleLines (getConvexEdges (), numSampledPoints);
      }

      typename PointCloud<PointT>::Ptr getSampledConvexEdges (float regularity)
      {
        return sampleLines (getConvexEdges (), regularity);
      }

      typename PointCloud<PointT>::Ptr getSampledConcaveEdges (int numSampledPoints)
      {
        return sampleLines (getConcaveEdges (), numSampledPoints);
      }

      typename PointCloud<PointT>::Ptr getSampledConcaveEdges (float regularity)
      {
        return sampleLines (getConcaveEdges (), regularity);
      }

      typename PointCloud<PointT>::Ptr getSampledLeftTreadEdges (int numSampledPoints)
      {
        typename PointCloud<PointT>::Ptr output (new PointCloud<PointT>);

        const StepsVector& steps = localModel.getSteps ();
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasTread ())
          {
            Tread<PointT> tread = steps[i].getTread ();
            BoundingBox<PointT> bbox = tread.getBBox ();
            typename PointCloud<PointT>::Ptr lineCloud = reconstructLineSegment (bbox.topLeft, bbox.bottomLeft,
                numSampledPoints);
//            printf("numPoints=%d: ", lineCloud->size());
//            for (size_t i = 0; i < lineCloud->size(); i++) {
//              PointT& p = lineCloud->at(i);
//              printf("(%f,%f,%f),", p.x,p.y, p.z);
//            }
//            printf("\n");
            concatePointClouds (*lineCloud, *output);
          }
        }
        return output;
      }
      typename PointCloud<PointT>::Ptr getSampledRightTreadEdges (int numSampledPoints)
      {
        typename PointCloud<PointT>::Ptr output (new PointCloud<PointT>);

        const StepsVector& steps = localModel.getSteps ();
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasTread ())
          {
            Tread<PointT> tread = steps[i].getTread ();
            BoundingBox<PointT> bbox = tread.getBBox ();
            typename PointCloud<PointT>::Ptr lineCloud = reconstructLineSegment (bbox.topRight, bbox.bottomRight,
                numSampledPoints);
            concatePointClouds (*lineCloud, *output);
          }
        }
        return output;
      }
      typename PointCloud<PointT>::Ptr getSampledLeftRiserEdges (int numSampledPoints = 5)
      {
        typename PointCloud<PointT>::Ptr output (new PointCloud<PointT>);

        const StepsVector& steps = localModel.getSteps ();
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasRiser ())
          {
            Riser<PointT> riser = steps[i].getRiser ();
            BoundingBox<PointT> bbox = riser.getBBox ();
            typename PointCloud<PointT>::Ptr lineCloud = reconstructLineSegment (bbox.topLeft, bbox.bottomLeft,
                numSampledPoints);
            concatePointClouds (*lineCloud, *output);
          }
        }
        return output;
      }
      typename PointCloud<PointT>::Ptr getSampledRightRiserEdges (int numSampledPoints = 5)
      {
        typename PointCloud<PointT>::Ptr output (new PointCloud<PointT>);

        const StepsVector& steps = localModel.getSteps ();
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasRiser ())
          {
            Riser<PointT> riser = steps[i].getRiser ();
            BoundingBox<PointT> bbox = riser.getBBox ();
            typename PointCloud<PointT>::Ptr lineCloud = reconstructLineSegment (bbox.topRight, bbox.bottomRight,
                numSampledPoints);
            concatePointClouds (*lineCloud, *output);
          }
        }
        return output;
      }
      typename PointCloud<PointT>::Ptr getSampledLeftEdges (int numSampledPoints = 5)
      {
        typename PointCloud<PointT>::Ptr leftTreads, leftRisers, output (new PointCloud<PointT>);
        leftTreads = getSampledLeftTreadEdges (numSampledPoints);
        leftRisers = getSampledLeftRiserEdges (numSampledPoints);
        concatePointClouds (*leftTreads, *output);
        concatePointClouds (*leftRisers, *output);
        return output;
      }
      typename PointCloud<PointT>::Ptr getSampledRightEdges (int numSampledPoints = 5)
      {
        typename PointCloud<PointT>::Ptr rightTreads, rightRisers, output (new PointCloud<PointT>);
        rightTreads = getSampledRightTreadEdges (numSampledPoints);
        rightRisers = getSampledRightRiserEdges (numSampledPoints);
        concatePointClouds (*rightTreads, *output);
        concatePointClouds (*rightRisers, *output);
        return output;
      }

      typename PointCloud<PointT>::Ptr getTreadsPoints ()
      {
        StepsVector steps = localModel.getSteps ();
        typename PointCloud<PointT>::Ptr outCloud (new PointCloud<PointT>);
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasTread ())
          {
            typename PointCloud<PointT>::Ptr treadCloud = steps[i].getTread ().getCloud ();
            outCloud->insert (outCloud->end (), treadCloud->begin (), treadCloud->end ());
          }
        }
        return outCloud;
      }

      typename PointCloud<PointT>::Ptr getRisersPoints ()
      {
        StepsVector steps = localModel.getSteps ();
        typename PointCloud<PointT>::Ptr outCloud (new PointCloud<PointT>);
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasRiser ())
          {
            typename PointCloud<PointT>::Ptr riserCloud = steps[i].getRiser ().getCloud ();
            outCloud->insert (outCloud->end (), riserCloud->begin (), riserCloud->end ());
          }
        }
        return outCloud;
      }

      typename PointCloud<PointT>::Ptr getTreadsConvexHulls ()
      {
        StepsVector steps = localModel.getSteps ();
        typename PointCloud<PointT>::Ptr outCloud (new PointCloud<PointT>);
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasTread ())
          {
            const Tread<PointT>& tread = steps[i].getTread ();
            typename PointCloud<PointT>::Ptr cloud = tread.getConvexHull ();
            outCloud->insert (outCloud->end (), cloud->begin (), cloud->end ());
          }
        }
        return outCloud;
      }

      typename PointCloud<PointT>::Ptr getRisersConvexHulls ()
      {
        StepsVector steps = localModel.getSteps ();
        typename PointCloud<PointT>::Ptr outCloud (new PointCloud<PointT>);
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasRiser ())
          {
            const Riser<PointT>& riser = steps[i].getRiser ();
            typename PointCloud<PointT>::Ptr cloud = riser.getConvexHull ();
            outCloud->insert (outCloud->end (), cloud->begin (), cloud->end ());
          }
        }
        return outCloud;
      }

      typename PointCloud<PointT>::Ptr getSampledBottomTreadEdges (float regularity)
      {
        typename PointCloud<PointT>::Ptr output (new PointCloud<PointT>);

        const StepsVector& steps = localModel.getSteps ();
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasTread ())
          {
            Tread<PointT> tread = steps[i].getTread ();
            BoundingBox<PointT> bbox = tread.getBBox ();
            typename PointCloud<PointT>::Ptr lineCloud = reconstructLineSegment (bbox.bottomLeft, bbox.bottomRight,
                regularity);
            concatePointClouds (*lineCloud, *output);
          }
        }
        return output;
      }

      typename PointCloud<PointT>::Ptr getSampledTopTreadEdges (float regularity)
      {
        typename PointCloud<PointT>::Ptr output (new PointCloud<PointT>);

        const StepsVector& steps = localModel.getSteps ();
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasTread ())
          {
            Tread<PointT> tread = steps[i].getTread ();
            BoundingBox<PointT> bbox = tread.getBBox ();
            typename PointCloud<PointT>::Ptr lineCloud = reconstructLineSegment (bbox.topLeft, bbox.topRight,
                regularity);
            concatePointClouds (*lineCloud, *output);
          }
        }
        return output;
      }

      typename PointCloud<PointT>::Ptr getSampledTopRiserEdges (float regularity)
      {
        typename PointCloud<PointT>::Ptr output (new PointCloud<PointT>);

        const StepsVector& steps = localModel.getSteps ();
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasRiser ())
          {
            Riser<PointT> riser = steps[i].getRiser ();
            BoundingBox<PointT> bbox = riser.getBBox ();
            typename PointCloud<PointT>::Ptr lineCloud = reconstructLineSegment (bbox.topLeft, bbox.topRight,
                regularity);
            concatePointClouds (*lineCloud, *output);
          }
        }
        return output;
      }

      typename PointCloud<PointT>::Ptr getSampledBottomRiserEdges (float regularity)
      {
        typename PointCloud<PointT>::Ptr output (new PointCloud<PointT>);

        const StepsVector& steps = localModel.getSteps ();
        for (size_t i = 0; i < steps.size (); i++)
        {
          if (steps[i].hasRiser ())
          {
            Riser<PointT> riser = steps[i].getRiser ();
            BoundingBox<PointT> bbox = riser.getBBox ();
            typename PointCloud<PointT>::Ptr lineCloud = reconstructLineSegment (bbox.bottomLeft, bbox.bottomRight,
                regularity);
            concatePointClouds (*lineCloud, *output);
          }
        }
        return output;
      }
  };
}

#define PCL_INSTANTIATE_ModelSampler(T) template class PCL_EXPORTS pcl::ModelSampler<T>;

#endif /* LOCALMODELSAMPLER_H_ */
