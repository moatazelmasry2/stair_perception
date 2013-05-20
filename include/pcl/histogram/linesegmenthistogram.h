/*
 * histogram.h
 *
 *  Created on: Apr 19, 2012
 *      Author: elmasry
 *      @deprecated class
 */

#ifndef LINESEGMENT_HISTOGRAM_H_
#define LINESEGMENT_HISTOGRAM_H_

#include <vector>

#include <pcl/point_cloud.h>

#include "pcl/types/linesegment3d.h"
#include <pcl/common/math.h>

namespace pcl
{

  template<typename PointT>
    class LineSegmentBin
    {
      typedef Eigen::aligned_allocator<PointT> Alloc;
      typedef std::vector<LineSegment3D<PointT> , Alloc> LineVector;
    protected:
      float rangeMin, rangeMax;int id;
      LineVector lines;

    public:
      LineSegmentBin (int id, float rangeMin, float rangeMax)
      {
        this->id = id;
        this->rangeMin = rangeMin;
        this->rangeMax = rangeMax;
      }

      LineSegmentBin (const LineSegmentBin<PointT>& bin)
      {
        rangeMin = bin.rangeMin;
        rangeMax = bin.rangeMax;
        id = bin.id;
        lines = bin.lines;
      }

      LineSegmentBin<PointT>&
      operator= (const LineSegmentBin<PointT>& bin)
      {
        rangeMin = bin.rangeMin;
        rangeMax = bin.rangeMax;
        id = bin.id;
        lines = bin.lines;
        return (*this);
      }
      inline float
      getRangeMin ()
      {
        return rangeMin;
      }

      inline float
      getRangeMax ()
      {
        return rangeMax;
      }

      inline int
      getId ()
      {
        return id;
      }

      LineVector
      getLines ()
      {
        return lines;
      }

      void
      addLine (LineSegment3D<PointT> line)
      {
        lines.push_back (line);
      }

    };

  template<typename PointT>
    class LineSegmentHistogram
    {

      typedef Eigen::aligned_allocator<PointT> Alloc;
      typedef std::vector<LineSegmentBin<PointT> , Alloc> BinsVector;
      typedef std::vector<LineSegment3D<PointT> , Alloc> LineVector;
    protected:

      BinsVector bins;
      LineVector lines;
      float min, max;

      /**
       * extracts min and max z of lines (camera coordinates)
       */
      void
      getMinMax (float& minX, float& maxX)
      {
        for (size_t i = 0; i < lines.size (); i++)
        {
          LineSegment3D<PointT>& line = lines[i];
          if (line.getCentroid ().z > maxX)
          {
            maxX = line.getCentroid ().z;
          }

          if (line.getCentroid ().z < minX)
          {
            minX = line.getCentroid ().z;
          }
        }
      }

      void
      initBins ()
      {
        float step = 0.2;
        float tmpMin = min, tmpMax = tmpMin + step;
        int count = 0;
        while (tmpMax <= max)
        {
          LineSegmentBin<PointT> bin (count++, tmpMin, tmpMax);
          bins.push_back (bin);
          tmpMin = tmpMax;
          tmpMax += step;
        }
        LineSegmentBin<PointT> bin (count++, tmpMin, tmpMax);
        bins.push_back (bin);
      }

      int
      findBin (BinsVector __bins, float xValue)
      {
        if ((int)__bins.size () == 1)
        {
          LineSegmentBin<PointT> b = __bins[0];
          if (xValue >= b.getRangeMin () && xValue <= b.getRangeMax ())
          {
            return b.getId ();
          }
          else
          {
            return -1;
          }
        }

        LineSegmentBin<PointT> bin = __bins[(int)__bins.size () / 2];
        if (xValue < bin.getRangeMin () && __bins.size () > 1)
        {
          typename BinsVector::iterator begin = __bins.begin ();
          typename BinsVector::iterator end = __bins.begin () + (int)__bins.size () / 2;
          BinsVector list;
          list.assign (begin, end);
          if (list.size () == 0)
          {
            return -1;
          }
          return findBin (list, xValue);
        }
        else if (xValue > bin.getRangeMax () && __bins.size () > 1)
        {
          typename BinsVector::iterator begin = __bins.begin () + (int)__bins.size () / 2;
          typename BinsVector::iterator end = __bins.end ();
          BinsVector list;
          list.insert (list.end (), begin, end);
          if (list.size () == 0)
          {
            return -1;
          }
          return findBin (list, xValue);
        }
        else if (xValue >= bin.getRangeMin () && xValue <= bin.getRangeMax ())
        {
          return bin.getId ();
        }
        else
        {
          return -1;
        }
      }

      void
      addLinesToHistogram ()
      {
        for (size_t i = 0; i < lines.size (); i++)
        {
          LineSegment3D<PointT>& line = lines[i];
          int index = findBin (bins, line.getCentroid ().z);
          if (index > 0)
          {
            LineSegmentBin<PointT>& bin = bins[index];
            bin.addLine (line);
          }
        }
      }

      void
      mergeLines (LineSegmentBin<PointT>& bin)
      {
        LineVector lines = bin.getLines ();
        std::vector<int> ledger ((int)lines.size (), 1);
        int merges = 0;
        for (size_t i = 0; i < lines.size (); i++)
        {
          if (ledger[i] < 0)
          {
            continue;
          }
          LineSegment3D<PointT>& line1 = lines[i];
          for (size_t j = i + 1; j < lines.size (); j++)
          {
            if (ledger[j] < 0)
            {
              continue;
            }
            LineSegment3D<PointT>& line2 = lines[j];
            float angle = pcl::getAngle (line1.getNormal (), line2.getNormal ());
            float yDistance = fabs (line1.getCentroid ().y - line2.getCentroid ().y);
            float zDistance = fabs (line1.getCentroid ().z - line2.getCentroid ().z);
            float maxDist = 0.4;
            if (angle < 20 && yDistance < maxDist && zDistance < maxDist)
            {
              merges++;
              typename pcl::PointCloud<PointT>::Ptr input = line1.getCloud ();
              typename pcl::PointCloud<PointT>::Ptr cl2 = line2.getCloud ();
              input->points.insert (input->end (), cl2->begin (), cl2->end ());
              line1.setInputCloud (input);
              ledger[j] = -1;
            }
          }

          LineVector tmpLines;
          for (size_t i = 0; i < lines.size (); i++)
          {
            if (ledger[i] >= 0)
            {
              tmpLines.push_back (lines[i]);
            }
          }
          lines.clear ();
          lines.assign (tmpLines.begin (), tmpLines.end ());
          std::cout << "numMerges=" << merges << std::endl;
        }
      }

    public:

      LineSegmentHistogram ()
      {

      }

      void
      addLine (LineSegment3D<PointT> line)
      {
        lines.push_back (line);
      }

      void
      addLines (LineVector lines)
      {
        this->lines.insert (this->lines.end (), lines.begin (), lines.end ());
      }

      void
      calculate ()
      {
        bins.clear ();
        min = 100, max = -100;
        getMinMax (min, max);
        //step size = 20cm
        initBins ();
        addLinesToHistogram ();
        //std::cout << "min=" << min << ", max=" << max;
        for (size_t i = 0; i < bins.size (); i++)
        {
          LineSegmentBin<PointT>& bin = bins[i];
          mergeLines (bin);
        }

      }

      LineVector
      getLines ()
      {
        LineVector outLines;
        lines.clear ();
        for (size_t i = 0; i < bins.size (); i++)
        {
          LineVector ls = bins[i].getLines ();
          outLines.insert (outLines.end (), ls.begin (), ls.end ());
        }
        return outLines;
      }

    };
}

#define PCL_INSTANTIATE_LineSegmentBin(T) template class PCL_EXPORTS pcl::LineSegmentBin<T>;
#define PCL_INSTANTIATE_LineSegmentHistogram(T) template class PCL_EXPORTS pcl::LineSegmentHistogram<T>;
#endif /* HISTOGRAM_H_ */
