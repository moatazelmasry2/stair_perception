/*
 * planehistogram.h
 *
 *  Created on: Jul 4, 2012
 *      Author: elmasry
 */

#ifndef PLANEHISTOGRAM_H_
#define PLANEHISTOGRAM_H_

#include <vector>
#include <map>
#include <Eigen/Dense>

#include "pcl/types/plane3d.h"
#include "pcl/histogram/abstract_histogram.h"

namespace pcl
{
  /**
   * type T must be a derived type of Plane3D
   */
  template<typename T, typename PointT>
  class PlaneHistogram : public AbstractHistogram<T, 1>
  {
      typedef Eigen::aligned_allocator<Plane3D<PointT> > Alloc;
      typedef std::vector<T, Alloc> PlanesVector;
      typedef std::vector<Bin<T, 0>, Alloc> BinsVector;
      typedef Bin<T, 0> PlaneBin;

    protected:
      int compareIndex;
      PlanesVector planes, outPlanes;
      Bin<T, 1> matrix;
      float stepSize;
      int numBins;

      void findMinMax (float& min, float& max)
      {
        typename std::map<double, T>::iterator mapIt;

        min = 99999;
        max = -99999;

        for (typename PlanesVector::iterator it = planes.begin (); it != planes.end (); it++)
        {
          T& plane = *it;
          if (plane.getCenter ()[compareIndex] < min)
          {
            min = plane.getCenter ()[compareIndex];
          }
          if (plane.getCenter ()[compareIndex] > max)
          {
            max = plane.getCenter ()[compareIndex];
          }
        }
      }
      void createHistogram (float min, float max, float stepSize, int numBins)
      {
        //printf("createHistogram: min=%f, max=%f, stepSize=%f, numBins=%d\n", min, max, stepSize, numBins);
        float rangeMin = min, rangeMax = min + stepSize;

        matrix.rangeMin = min;
        matrix.rangeMax = max;
        for (int i = 0; i < numBins; i++)
        {
          Bin<T, 0> bin (i, rangeMin, rangeMax);
          matrix.push_back (bin);
          rangeMin = rangeMax;
          rangeMax = rangeMin + stepSize;
        }
      }

      /**
       * Search for the right bin using binary search
       */
      int findBin2 (BinsVector __bins, double value)
      {

        if (__bins.size () == 1)
        {
          PlaneBin& binT = __bins[(int) __bins.size () / 2];
          if (value >= binT.rangeMin && value <= binT.rangeMax)
          {
            return binT.index;
          }
          else
          {
            return -1;
          }
        }
        if (__bins.size () == 0)
        {
          return -1;
        }

        PlaneBin& bin = __bins[(int) __bins.size () / 2];
        if (value < bin.rangeMin)
        {
          typename BinsVector::iterator begin = __bins.begin ();
          typename BinsVector::iterator end = __bins.begin () + (int) __bins.size () / 2;
          BinsVector list;
          list.assign (begin, end);
          return findBin (list, value);
        }
        else if (value > bin.rangeMax)
        {
          typename BinsVector::iterator begin = __bins.begin () + (int) __bins.size () / 2;
          typename BinsVector::iterator end = __bins.end ();
          BinsVector list;
          list.insert (list.end (), begin, end);
          return findBin (list, value);
        }
        return bin.index;;
      }

      /**
       * Search for the right bin using binary search
       */
      int findBin (BinsVector __bins, float value)
      {
        if (__bins.size () == 0)
          return -1;

        if (__bins.size () == 1)
        {
          PlaneBin& binT = __bins[(int) (__bins.size () / 2)];
          if (value >= binT.rangeMin && value <= binT.rangeMax)
          {
            return binT.index;
          }
          else
          {
            return -1;
          }
        }

        PlaneBin& bin = __bins[(int) __bins.size () / 2];
        if (value < bin.rangeMin)
        {
          typename BinsVector::iterator begin = __bins.begin ();
          typename BinsVector::iterator end = __bins.begin () + (int) (__bins.size () / 2);
          BinsVector list;
          list.assign (begin, end);
          return findBin (list, value);
        }
        else if (value > bin.rangeMax)
        {
          typename BinsVector::iterator begin = __bins.begin () + (int) (__bins.size () / 2);
          typename BinsVector::iterator end = __bins.end ();
          BinsVector list;
          list.insert (list.end (), begin, end);
          return findBin (list, value);
        }
        return bin.index;
      }

      void addData ()
      {
        for (typename PlanesVector::iterator it = planes.begin (); it != planes.end (); it++)
        {
          T& plane = *it;
          float value = plane.getCenter ()[compareIndex];
          int index = findBin (matrix.bins, value);
          if (index < 0)
          {
            PCL_WARN("In Histogram a value has not been found: %f\n", value);
            continue;
          }
          matrix[index].push_back (plane);
        }
      }

      void fillOutPlanes ()
      {
        outPlanes.clear ();
        int count = 0;
        for (size_t i = 0; i < matrix.bins.size (); i++)
        {
          for (size_t j = 0; j < matrix.bins[i].content.size (); j++)
          {
            count++;
            outPlanes.push_back (matrix.bins[i][j]);
          }
        }
      }

      virtual void mergePlanes ()
      {
        outPlanes.clear ();
        for (size_t i = 0; i < matrix.bins.size (); i++)
        {
          PlaneBin& bin = matrix.bins[i];
          T outPlane;
          //printf (", merge: (");
          for (size_t j = 0; j < bin.content.size (); j++)
          {
            T& plane = bin[j];
            //printf ("%d,", plane.id);
            if (j == 0)
            {
              outPlane = plane;
            }
            else
            {
              if (areCloseNeighbors (outPlane, plane))
              {
                outPlane += plane;
                //printf("mergePlanes %d,%d\n", outPlane.getId(), plane.getId());
                //outPlane.logPlane();
              }

            }
          }

          if (bin.content.size () > 0)
          {
            bin.content.clear ();
            //outPlane.id = i;
            bin.content.push_back (outPlane);

            //merge content from neighboring bins if close enough
            bool binsMerge = true;
            //if i > 0, means if not the first element, and thus no comparison to upper elements
            if (i > 0 && matrix[i - 1].content.size () > 0 && binsMerge)
            {
              T& p1 = matrix.bins[i - 1][0];
              //printf ("*%d", p1.id);
              T& p2 = bin[0];
              if (fabs (p1.getCenter ()[compareIndex] - p2.getCenter ()[compareIndex]) < maxDistanceThreshold
                  && areCloseNeighbors (p1, p2))
              {
                p1 += p2;
                bin.content.clear ();
              }
            }
          }
          //printf (")");
        }
        fillOutPlanes ();
      }

      void initHistogram (float stepSize)
      {
        float min, max;
        findMinMax (min, max);
        //printf ("findMinmax: min=%f, max=%f\n", min, max);
        int numBins = (max - min) / stepSize + 1;
        createHistogram (min, max, stepSize, numBins);
      }

      void initHistogram (int numBins)
      {
        float min, max;
        findMinMax (min, max);
        //printf ("findMinmax: min=%f, max=%f\n", min, max);
        float stepSize = (max - min) / numBins;
        createHistogram (min, max, stepSize, numBins);
      }

      void initHistogram ()
      {
        float min, max;
        findMinMax (min, max);
        int numBins = (int) ( (max - min) / stepSize + 1);
        createHistogram (min, max, stepSize, numBins);
      }

      bool areCloseNeighbors (const Plane3D<PointT>& plane1, const Plane3D<PointT>& plane2)
      {
        //max distance allowed between two non intersecting planes
        float maxDistance = 0.04;

        float left1 = (plane1.getBBox ().topLeft.y + plane1.getBBox ().bottomLeft.y) / 2;
        float right1 = (plane1.getBBox ().topRight.y + plane1.getBBox ().bottomRight.y) / 2;
        float left2 = (plane2.getBBox ().topLeft.y + plane2.getBBox ().bottomLeft.y) / 2;
        float right2 = (plane2.getBBox ().topRight.y + plane2.getBBox ().bottomRight.y) / 2;

        if (plane1.isTread () && plane2.isTread ())
        {
          float top1 = (plane1.getBBox ().topLeft.x + plane1.getBBox ().topRight.z) / 2;
          float bottom1 = (plane1.getBBox ().bottomLeft.x + plane1.getBBox ().bottomRight.x) / 2;
          float top2 = (plane2.getBBox ().topLeft.x + plane2.getBBox ().topRight.x) / 2;
          float bottom2 = (plane2.getBBox ().bottomLeft.x + plane2.getBBox ().bottomRight.x) / 2;

          float diff1 = top1 - bottom2;
          float diff2 = bottom1 - top2;
          //if the two subtractions have different sign, then there's an intersection, we continue
          //if they have same sign, then no intersection and should check they are close enough
          if ( (diff1 > 0 && diff2 > 0) || (diff1 < 0 && diff2 < 0))
          {
            float minX = fabs (diff1) < fabs (diff2) ? fabs (diff1) : fabs (diff2);
            if (minX > maxDistance)
              return false;
          }

          diff1 = left1 - right2;
          diff2 = right1 - left2;
          if ( (diff1 > 0 && diff2 > 0) || (diff1 < 0 && diff2 < 0))
          {
            float minX = fabs (diff1) < fabs (diff2) ? fabs (diff1) : fabs (diff2);
            if (minX > maxDistance)
              return false;
          }
          return true;
        }
        else if (plane1.isRiser () && plane2.isRiser ())
        {
          float top1 = (plane1.getBBox ().topLeft.z + plane1.getBBox ().topRight.z) / 2;
          float bottom1 = (plane1.getBBox ().bottomLeft.z + plane1.getBBox ().bottomRight.z) / 2;
          float top2 = (plane2.getBBox ().topLeft.z + plane2.getBBox ().topRight.z) / 2;
          float bottom2 = (plane2.getBBox ().bottomLeft.z + plane2.getBBox ().bottomRight.z) / 2;

          float diff1 = top1 - bottom2;
          float diff2 = bottom1 - top2;
          //if the two subtractions have different sign, then there's an intersection, we continue
          //if they have same sign, then no intersection and should check they are close enough
          if ( (diff1 > 0 && diff2 > 0) || (diff1 < 0 && diff2 < 0))
          {
            float minX = fabs (diff1) < fabs (diff2) ? fabs (diff1) : fabs (diff2);
            if (minX > maxDistance)
              return false;
          }

          diff1 = left1 - right2;
          diff2 = right1 - left2;
          if ( (diff1 > 0 && diff2 > 0) || (diff1 < 0 && diff2 < 0))
          {
            float minX = fabs (diff1) < fabs (diff2) ? fabs (diff1) : fabs (diff2);
            if (minX > maxDistance)
              return false;
          }
          return true;
        }

        return false;
      }

    public:

      //maximum distance allowed between two planes to merge them
      float maxDistanceThreshold;
      /**
       * On which index to compare the planes, x,y,z corresponds to 0,1,2
       */
      PlaneHistogram (int compareIndex)
      {
        this->compareIndex = compareIndex;
        stepSize = -1;
        numBins = -1;
        //maxDistanceThreshold = 50.0f;
      }

      void logHistogramStructure ()
      {
        printf ("Log Histogram structure: numbins=%d\n", (int) matrix.bins.size ());
        printf ("min=%f, max=%f\n", matrix.rangeMin, matrix.rangeMax);
        for (size_t i = 0; i < matrix.bins.size (); i++)
        {
          Bin<T, 0>& bin = matrix.bins[i];
          printf ("bin%d: min=%f, max=%f\n", bin.index, bin.rangeMin, bin.rangeMax);
        }
      }

      void logContent ()
      {
        printf ("numbins=%d\n", (int) matrix.bins.size ());
        for (size_t i = 0; i < matrix.bins.size (); i++)
        {
          Bin<T, 0>& bin = matrix.bins[i];
          printf ("bin%d: planes=", bin.index);
          for (size_t j = 0; j < bin.content.size (); j++)
          {
            T& plane = bin[j];
            printf ("%d,", plane.getId ());
          }
          printf ("\n");
        }
      }

      void setStepsize (float stepSize)
      {
        this->stepSize = stepSize;
        numBins = -1;
      }

      void setNumBins (int numBins)
      {
        this->numBins = numBins;
        stepSize = -1;
      }

      void addPlane (const T& plane)
      {
        planes.push_back (plane);
      }

      void addPlanes (PlanesVector planes)
      {
        this->planes.insert (this->planes.end (), planes.begin (), planes.end ());
      }

      bool compute ()
      {
        //printf ("Histogram: numPlanes=%d\n", (int)planes.size ());
        if (planes.size () == 0)
          return false;

        if (planes.size () >= 2)
        {
          if (stepSize > 0)
          {
            initHistogram (stepSize);
          }
          else if (numBins > 0)
          {
            initHistogram (numBins);
          }
          else
          {
            initHistogram ();
          }

          //logHistogramStructure ();
          addData ();
          //logContent ();
          //printf ("before merge:\n");
          mergePlanes ();

          //printf ("after merge:\n");
          //logContent ();

        }
        else
        {
          outPlanes = planes;
        }
        return true;
      }

      PlanesVector getOutplanes ()
      {
        return outPlanes;
      }

  };
}

#endif /* PLANEHISTOGRAM_H_ */
