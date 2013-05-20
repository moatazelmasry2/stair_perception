/*
 * globfitwriter.h
 *
 *  Created on: Jan 25, 2013
 *      Author: elmasry
 */

#ifndef GLOBFITWRITER_H_
#define GLOBFITWRITER_H_

#include <iostream>
#include <fstream>

#include <vector>

#include <pcl/point_cloud.h>

#include "pcl/models/localmodel.h"

namespace pcl
{
  namespace io
  {

    template<typename PointT>
    class GlobfitWriter
    {
      private:
        typedef Eigen::aligned_allocator<PointT> Alloc;
        typedef std::vector<Step<PointT>, Alloc> StepsVector;
        typedef std::vector<pcl::Plane3D<PointT>, Alloc> Plane3DVector;

        typename pcl::PointCloud<PointT>::Ptr outCloud;
        LocalModel<PointT> model;

        void writePlane (pcl::Plane3D<PointT> plane, std::ofstream& myfile)
        {
          myfile << "# plane normal_x normal_y normal_z d" << std::endl;
          Eigen::Vector3f normal = plane.getNormal ();
          myfile << "plane " << normal[0] << " " << normal[1] << " " << normal[2] << " " << plane.getHesseDistance ()
              << std::endl;
          myfile << "# points idx_1 idx_2 idx_3 ... " << std::endl;
          typename PointCloud<PointT>::Ptr cloud = plane.getCloud ();
          myfile << "points ";
          for (size_t i = 0; i < cloud->size (); i++)
          {
            myfile << (*cloud)[i].id;
            if (i + 1 < cloud->size ())
            {
              myfile << " ";
            }
          }
          myfile << std::endl;
        }
        void writePlanes (const Plane3DVector& planes, std::ofstream& myfile)
        {
          for (size_t i = 0; i < planes.size (); i++)
          {
            myfile << "#primitive: " << i << std::endl;
            writePlane (planes[i], myfile);
          }
        }

        void writeCloud (typename PointCloud<PointT>::Ptr cloud, std::ofstream& myfile)
        {
          myfile << "#number of points" << std::endl;
          myfile << cloud->size () << std::endl;
          myfile << "# Here comes the " << cloud->size () << " Points" << std::endl;
          myfile << "# point_x point_y point_z normal_x normal_y normal_z confidence" << std::endl;

          for (size_t i = 0; i < cloud->size (); i++)
          {
            const PointT& p = (*cloud)[i];
            if (::pcl::isNanPoint (p))
            {
              myfile << "0.0 0.0 0.0 0.0 0.0 0.0 1" << std::endl;
            }
            else
            {
              myfile << p.x << " " << p.y << " " << p.z << " " << p.normal_x << " " << p.normal_y << " " << p.normal_z
                  << " " << p.confidence << std::endl;
            }
          }
          myfile << "# End of Points" << std::endl;
        }

      protected:

        /**
         * Create an output cloud from the inputCloud
         */
        typename pcl::PointCloud<PointT>::Ptr createOutputCloud ()
        {
          const float DISTANCE_CONFIDENCE_THRESHOLD = 0.03;
          typename pcl::PointCloud<PointT>::Ptr outCloud (new pcl::PointCloud<PointT>);

          size_t counter = 0;
          for (size_t i = 0; i < model.steps.size (); i++)
          {
            Step<PointT>& step = model.steps[i];

            if (step.hasTread ())
            {
              Tread<PointT> tread = step.getTread ();
              typename pcl::PointCloud<PointT>::Ptr cloud = tread.getCloud ();
              for (size_t j = 0; j < cloud->size (); j++)
              {
                (*cloud)[j].id = counter++;
                //calculating confidende
                float dNext = 9999, dPrev = 9999;
                if ( (i < (model.steps.size () - 1)) && (model.steps[i + 1].hasRiser ()))
                {
                  dNext = fabs ( (*cloud)[j].x - model.steps[i + 1].getRiser ().getCenter()[0]);
                }
                if (i > 0 && (step.hasRiser ()))
                {
                  dPrev = fabs ( (*cloud)[j].x - step.getRiser ().getCenter()[0]);
                }

                float d1 = min (dPrev, dNext);
                float d2 = fabs ( (*cloud)[j].z - tread.getCenter()[2]);
                if (max (d1, d2) < DISTANCE_CONFIDENCE_THRESHOLD)
                {
                  (*cloud)[j].confidence = fabs (d1 - d2) / DISTANCE_CONFIDENCE_THRESHOLD;
                }
                outCloud->push_back ( (*cloud)[j]);
              }

              tread.setInputCloud (cloud, false, false);
              step.setTread (tread);
              model.steps[i] = step;
            }
            if (step.hasRiser ())
            {
              Riser<PointT> riser = step.getRiser ();
              typename pcl::PointCloud<PointT>::Ptr cloud = riser.getCloud ();
              for (size_t j = 0; j < cloud->size (); j++)
              {
                (*cloud)[j].id = counter++;
                float dNext = 9999, dPrev = 9999;
                if ( step.hasRiser () )
                {
                  dNext = fabs ( (*cloud)[j].z - step.getRiser ().getCenter()[2]);
                }
                if (i > 0 && (model.steps[i - 1].hasRiser ()))
                {
                  dPrev = fabs ( (*cloud)[j].z - model.steps[i - 1].getRiser ().getCenter()[2]);
                }

                float d1 = min (dPrev, dNext);
                float d2 = fabs ( (*cloud)[j].x - riser.getCenter()[0]);
                if (max (d1, d2) < DISTANCE_CONFIDENCE_THRESHOLD)
                {
                  (*cloud)[j].confidence = fabs (d1 - d2) / DISTANCE_CONFIDENCE_THRESHOLD;
                }
                outCloud->push_back ( (*cloud)[j]);
              }
              riser.setInputCloud (cloud, false, false);
              step.setRiser (riser);
              model.steps[i] = step;
            }
          }
          return outCloud;
        }

        Plane3DVector getTreads ()
        {
          Plane3DVector planes;
          const StepsVector& steps = model.getSteps ();
          for (size_t i = 0; i < steps.size (); i++)
          {
            const Step<PointT>& step = steps[i];
            if (step.hasTread ())
            {
              const Tread<PointT>& tread = step.getTread ();
              planes.push_back ((const Plane3D<PointT>&) tread);
            }
          }
          return planes;
        }

        Plane3DVector getRisers ()
        {
          Plane3DVector planes;
          const StepsVector& steps = model.getSteps ();
          for (size_t i = 0; i < steps.size (); i++)
          {
            const Step<PointT>& step = steps[i];
            if (step.hasRiser ())
            {
              const Riser<PointT>& riser = step.getRiser ();
              planes.push_back ((const Plane3D<PointT>&) riser);
            }
          }
          return planes;
        }

      public:

        void setModel (const LocalModel<PointT>& model)
        {
          this->model = model;
//          outCloud = createOutputCloud();
        }

        void writeGlobfitModel (char* outPath)
        {

          typename ::pcl::PointCloud<PointT>::Ptr outCloud = createOutputCloud ();
          Plane3DVector planes, risers;

          planes = getTreads ();
          risers = getRisers ();
          planes.insert (planes.end (), risers.begin (), risers.end ());

          if (planes.size () == 0)
          {
            std::cerr << "no treads found" << std::endl;
          }
          else
          {
            std::ofstream myfile;
            myfile.open (outPath);
            writeCloud (outCloud, myfile);
            myfile << "#number of primitives" << std::endl;
            myfile << planes.size () << std::endl;
            myfile << "# Here comes the " << planes.size () << " Primitives" << std::endl;
            writePlanes (planes, myfile);
            myfile << "# End of Primitives" << std::endl;
            myfile.close ();
          }
        }
    };
  }
}

#define PCL_INSTANTIATE_GlobfitWriter(T) template class PCL_EXPORTS pcl::io::GlobfitWriter<T>;

#endif /* GLOBFITWRITER_H_ */
