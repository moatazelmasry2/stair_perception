/*
 * stairdetectiondemo.h
 *
 *  Created on: Jul 15, 2012
 *      Author: elmasry
 */

#ifndef STAIRDETECTIONDEMO_H_
#define STAIRDETECTIONDEMO_H_

#include <Eigen/Dense>

#include "pcl/utils/pointcloud_utils.h"
#include "pcl/types/plane3d.h"
#include "pcl/segmentation/planesegmentation.h"
#include "pcl/common/color.h"

#include "pcl/common/fhg_point_types.h"
#include "pcl/common/edges_common.h"
#include "pcl/opencv/cvutils.h"
#include "pcl/opencv/edgesdetector.h"

#include "pcl/utils/pointcloud_utils.h"
#include "pcl/models/localmodel.h"
#include "pcl/models/localmodelfactory.h"
#include "pcl/models/edges2planesfactory.h"
#include "pcl/models/model_utils.h"
#include "pcl/models/globalmodel.h"
#include "pcl/io/globfitwriter.h"

namespace pcl
{
  template<typename PointIn, typename PointOut>
  class StairDetectionDemo
  {
      typedef Eigen::aligned_allocator<PointIn> Alloc;
      typedef Eigen::aligned_allocator<PointOut> AllocOut;
      typedef std::vector<pcl::Plane3D<PointOut>, AllocOut> Plane3DVector;
      typedef std::vector<pcl::LineSegment3D<PointOut>, Eigen::aligned_allocator<PointOut> > LineSegment3DVector;

      typename pcl::PointCloud<PointIn>::ConstPtr inCloud;
      pcl::PlaneSegmentation<PointIn, PointOut> p;
      float cameraAngle, cameraHeight;
      int numIterations;

    public:
      pcl::GlobalModel<PointOut> globalModel;
      std::string cloudName;

    protected:
      void logModel (LocalModel<PointOut> model)
      {
        std::vector<Step<PointOut>, Eigen::aligned_allocator<PointOut> > steps = model.getSteps ();
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
          Step<PointOut>& step = steps[i];
          if (step.hasTread ())
          {
            const Tread<PointOut>& tread = step.getTread ();
            printf ("Tread %f %f %f---\n", tread.getLength (), tread.getLDEpth (), tread.getRDepth ());
          }
          if (step.hasRiser ())
          {
            const Riser<PointOut>& riser = step.getRiser ();
            printf ("Riser %f %f %f---\n", riser.getLength (), riser.getLDEpth (), riser.getRDepth ());
          }
          printf ("\n");
        }
      }

      void calculateCameraHeight (const LocalModel<PointOut>& model)
      {
        if (model.getSteps ().size () > 0)
        {
          const Step<PointOut>& step = model.getSteps ()[0];
          if (model.isLookingUpstairs ())
          {
            if (step.hasRiser ())
            {
              cameraHeight = step.getRiser ().getHesseDistance ();
            }
            else
            {
              cameraHeight = step.getTread ().getHesseDistance ();
            }
          }
          else if (model.isLookingDownstairs ())
          {
            if (step.hasTread ())
            {
              cameraHeight = step.getTread ().getHesseDistance ();
            }
            else
            {
              cameraHeight = step.getRiser ().getHesseDistance ();
            }
          }
        }
      }

    public:

      StairDetectionDemo() {
        numIterations = 0;
        cameraHeight = 0.0f;
        cameraAngle = 0.0f;
      }
      void setInputCloud (const typename pcl::PointCloud<PointIn>::ConstPtr inCloud)
      {
        this->inCloud = inCloud;
      }

      /**
       */
      typename pcl::PointCloud<PointIn>::Ptr compute ()
      {
        typename pcl::PointCloud<PointIn>::Ptr cloud (new pcl::PointCloud<PointIn> ());
        pcl::copyPointCloud (*inCloud, *cloud);
        typename pcl::PointCloud<PointIn>::Ptr grey_input_cloud (new pcl::PointCloud<PointIn> ());
        pcl::copyPointCloud (*inCloud, *grey_input_cloud);
        pcl::io::savePCDFileASCII ("incloud_gray.pcd", *grey_input_cloud);

        cloud->width = inCloud->width;
        cloud->height = inCloud->height;
        Quaternion<float> rotQuaternion;
        pcl::cameraToworld (*cloud);

        typename pcl::PointCloud<PointIn>::Ptr outCloud (new pcl::PointCloud<PointIn> ());

        //use Dirk's Region Growth

        p.setInputCloud (cloud);
        p.compute ();
        cameraAngle = p.getRotationangle ();
        rotQuaternion = p.rotQuaternion;
        Plane3DVector planes = p.getPlanes ();

//        {
//          //  DEBUG /////
//          typename pcl::PointCloud<PointOut>::Ptr rotatedCloud (new pcl::PointCloud<PointOut>);
//          pcl::copyPointCloud (*cloud, *rotatedCloud);
//          pcl::rotatePointCloud (*rotatedCloud, rotQuaternion);
//          pcl::io::savePCDFileASCII ("rotated_cloud.pcd", *rotatedCloud);
//          float black = pcl::generateColor (0, 0, 0);
//          pcl::colorCloud (*rotatedCloud, black);
//          pcl::io::savePCDFileASCII ("rotated_cloud_black.pcd", *rotatedCloud);
//        }

        LocalModelFactory<PointOut> fac;
        fac.addPlanes (planes);
        LocalModel<PointOut> model = fac.createLocalModel (true);

        {
          typename pcl::PointCloud<PointOut>::Ptr bordersCloud = model.getBoundingBoxesCloud();
          pcl::colorCloud(*bordersCloud, pcl::generateColor(0,0,0));
          char f[50];
          sprintf(f, "borders_%d.pcd", numIterations);
          pcl::io::saveMoPcd(f, *bordersCloud);
          printf("localmodel after creation and add missing planes\n");
          model.logSteps();

        }
//        printf("localmodel before edge detection:\n");
//        model.logSteps();

        calculateCameraHeight (model);
        //float theta = 90 - cameraAngle;
        //cameraHeight = model.calculateCenterOfMass ()[0] * sin (theta * M_PI / 180);

        //////////////////////////////////////////////////////////////////////////////////////
        //////////////////////  EDGE DETECTION            ////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////

        typedef pcl::PointMoXYZRGB MoRGB;
        pcl::opencv::EdgeDetector<PointIn, PointOut> detector;
        LineSegment3DVector lines, linesFromNormalImg, linesfromDepthJumps;
        //rotate if isLookingDownstairs
        detector.setInputCloud (inCloud);
        linesFromNormalImg = detector.computeFromNormalImg ();
        for (size_t i = 0; i < linesFromNormalImg.size (); i++)
        {
          LineSegment3D<PointOut>& line = linesFromNormalImg[i];
          line.cameraToworld ();
          line.rotateLine (rotQuaternion);
        }

        typename pcl::PointCloud<PointIn>::Ptr tmpEdgesCloud (new pcl::PointCloud<PointIn>);
        pcl::copyPointCloud (*cloud, *tmpEdgesCloud);
        pcl::rotatePointCloud (*tmpEdgesCloud, rotQuaternion);
        typename pcl::PointCloud<PointIn>::ConstPtr tmpEdgesCloud2 (tmpEdgesCloud);
        detector.setInputCloud (tmpEdgesCloud2);
        linesfromDepthJumps = detector.computeFromDepthJumps ();
//        for (size_t i = 0; i < linesfromDepthJumps.size (); i++)
//        {
//          LineSegment3D<PointOut>& line = linesfromDepthJumps[i];
//          line.cameraToworld ();
//          line.rotateLine (rotQuaternion);
//        }

        //::pcl::logLines (detector.getVerticalLines ());

        lines.insert (lines.end (), linesFromNormalImg.begin (), linesFromNormalImg.end ());
        lines.insert (lines.end (), linesfromDepthJumps.begin (), linesfromDepthJumps.end ());

        //LineSegment3DHistogram<PointOut> hist;
        //hist.addLines(lines);
        //hist.mergeLines();
        //lines = hist.getLines();

        typename pcl::PointCloud<PointIn>::Ptr tmpCloud (new pcl::PointCloud<PointIn>);

        //create planes from edges
        Edges2PlanesFactory<PointOut> edges2planes;
        edges2planes.setEdges (lines);
        Plane3DVector planes2 = edges2planes.createPlanes ();

        {
          //////  DEBUG  ///////
//          typename pcl::PointCloud<PointOut>::Ptr bordersCloud = pcl::utils::sampleBoundingBoxes<PointOut> (planes2);
//          float green = pcl::generateColor (0, 255, 0);
//          pcl::colorCloud (*bordersCloud, green);
//          pcl::io::saveMoPcd ("sampled_edges_borders.pcd", *bordersCloud);
        }

//        fac.addPlanes (planes2);
//        fac.mergePlanes ();

        ///////////////////   CREATE WALLS   //////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        LineSegment3DVector verticalLines = detector.getVerticalLines ();
        Plane3DVector trails = ::pcl::model::lines2Trails (verticalLines);
        Plane3DVector walls = p.getWalls ();
        //walls.insert (walls.end (), trails.begin (), trails.end ());
//        fac.addWalls (walls);
//        fac.mergeWalls ();
        ///////////////////////////////////////////////////////////////////////////////////

        model = fac.createLocalModel ();

//        printf ("localmodel after adding edges:\n");
//        model.logSteps ();

//        ::pcl::io::GlobfitWriter<PointOut> writer;
//        writer.setModel (model);
//        writer.writeGlobfitModel ("myplanes.globfit");
//        globalModel.setCloudName (cloudName);
        globalModel.addLocalModel (model);
//        writer.setModel (globalModel);
//        writer.writeGlobfitModel ("globalmodel.globfit");

//        printf("localmodel after being added to global model:\n");
//        model.logSteps();
        //globalModel.logSteps();
        //model.logSteps ();

        //////  VISUALIZATION
        typename pcl::PointCloud<PointOut>::Ptr edgesCloud = pcl::lines2Cloud (lines);
        //add detected planes and created planes to model
        typename pcl::PointCloud<PointOut>::Ptr tmpOut = model.getModelCloud ();
        pcl::copyPointCloud (*tmpOut, *outCloud);

        {
          ////   DEBUG ////
          typename pcl::PointCloud<PointOut>::Ptr bordersCloud = model.getBoundingBoxesCloud();
          pcl::colorCloud(*bordersCloud, pcl::generateColor(255,0,0));
          pcl::io::saveMoPcd("model_borders.pcd", *bordersCloud);
        }

//        {
//          float color = pcl::generateColor (200, 200, 200);
//          typename pcl::PointCloud<PointOut>::Ptr grey_color (new pcl::PointCloud<PointOut>);
//          pcl::copyPointCloud (*getPlanesCloud (), *grey_color);
//          pcl::colorCloud (*grey_color, color);
//          pcl::io::saveMoPcd ("grey_cloud_planes.pcd", *grey_color);
//        }
//
//        {
//          float color = pcl::generateColor (200, 200, 200);
//          typename pcl::PointCloud<PointOut>::Ptr grey_color (new pcl::PointCloud<PointOut>);
//          pcl::copyPointCloud (*outCloud, *grey_color);
//          pcl::colorCloud (*grey_color, color);
//          pcl::io::saveMoPcd ("grey_cloud_localmodel.pcd", *grey_color);
//        }

//        typename pcl::PointCloud<PointIn>::Ptr edgesCloud2 (new pcl::PointCloud<PointIn>);
//        pcl::copyPointCloud (*edgesCloud, *edgesCloud2);
        //pcl::concatePointClouds (*edgesCloud2, *outCloud);

//        edgesCloud = pcl::lines2Cloud (detector.getVerticalLines ());
//        edgesCloud2->clear ();
//        pcl::copyPointCloud (*edgesCloud, *edgesCloud2);
        //pcl::concatePointClouds (*edgesCloud2, *outCloud);

        //model.logStepsModified ();
        printf ("----------------------------------------------------\n");

//                typename pcl::PointCloud<PointIn>::Ptr outCloud2(new pcl::PointCloud<PointIn>);
//                pcl::copyPointCloud(*inCloud, *outCloud2);
//                pcl::cameraToworld(*outCloud2);
//                pcl::rotatePointCloud(*outCloud2, rotQuaternion);
//                for (size_t i = 0; i < outCloud->size(); i++) {
//                    PointIn p1 = outCloud->points[i];
//                    if (p1.id < outCloud2->size() - 1) {
//                        outCloud2->points[p1.id] = p1;
//                    }
//                }
        numIterations++;
        return outCloud;
      }

      typename pcl::PointCloud<PointOut>::Ptr getPlanesCloud ()
      {
        return p.getOutputCloud ();
      }

      typename pcl::PointCloud<PointOut>::Ptr getRawPlanesCloud ()
      {
        return p.getRawPlanesCloud ();
      }

      GlobalModel<PointOut> getGlobalModel () const
      {
        return globalModel;
      }

      void setCloudName (std::string cloudName)
      {
        this->cloudName = cloudName;
      }

  };

}

#define PCL_INSTANTIATE_StairDetectionDemo(In,Out) template class PCL_EXPORTS pcl::StairDetectionDemo<In,Out>;
#endif /* STAIRDETECTIONDEMO_H_ */
