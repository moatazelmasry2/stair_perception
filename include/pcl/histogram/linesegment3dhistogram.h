/*
 * linesegment3dhistogram.h
 *
 *  Created on: Aug 7, 2012
 *      Author: elmasry
 */

#ifndef LINESEGMENT3DHISTOGRAM_H_
#define LINESEGMENT3DHISTOGRAM_H_

#include <Eigen/Dense>

#include "pcl/common/color.h"
#include "pcl/utils/pointcloud_utils.h"

#include "pcl/types/linesegment3d.h"
#include "pcl/histogram/abstract_histogram.h"

namespace pcl
{
  template<typename PointT>
    class LineSegment3DHistogram : public AbstractHistogram<LineSegment3D<PointT> , 4>
    {
      //max distance allowed between two lines
      static const float MAX_DIST_LINES = 0.04;
      //max difference between two normal components
      static const float MAX_ANGLE_BET_LINES = 10;

      typedef Eigen::aligned_allocator<LineSegment3D<PointT> > Alloc;
      typedef std::vector<LineSegment3D<PointT> , Alloc> LineVector;
      typedef std::vector<Bin<LineSegment3D<PointT> , 4> , Alloc> BinsVector;

      Eigen::Vector3f domNormal;

    protected:
      int compareIndex;
      LineVector lines, outLines;
      //matrix/histogram will be constructed in the following order: normal[0],normal[1],normal[2],height
      Bin<LineSegment3D<PointT> , 1> matrix;

      void
      findMinMax (float& min, float& max)
      {
        min = 99999;
        max = -99999;

        for (typename LineVector::iterator it = lines.begin (); it != lines.end (); it++)
        {
          LineSegment3D<PointT>& line = *it;
          if (line.getCenter ()[compareIndex] < min)
          {
            min = line.getCenter ()[compareIndex];
          }
          if (line.getCenter ()[compareIndex] > max)
          {
            max = line.getCenter ()[compareIndex];
          }
        }
      }

      /**
       * Search for the right bin using binary search
       */
      template<unsigned depth>
        int
        findBin (std::vector<Bin<LineSegment3D<PointT> , depth> , Alloc>& __bins, float value)
        {
          typedef std::vector<Bin<LineSegment3D<PointT> , depth> , Alloc> LocalBinsVector;
          if (__bins.size () == 1)
          {
            Bin<LineSegment3D<PointT> , depth> & binT = __bins[(int)__bins.size () / 2];
            if (value >= binT.rangeMin && value <= binT.rangeMax)
            {
              return binT.index;
            } else
            {
              return -1;
            }
          }
          if (__bins.size () == 0)
          {
            return -1;
          }

          Bin<LineSegment3D<PointT> , depth> & bin = __bins[(int)__bins.size () / 2];
          if (value < bin.rangeMin)
          {
            typename LocalBinsVector::iterator begin = __bins.begin ();
            typename LocalBinsVector::iterator end = __bins.begin () + (int)__bins.size () / 2;
            LocalBinsVector list;
            list.assign (begin, end);
            return findBin (list, value);
          } else if (value > bin.rangeMax)
          {
            typename LocalBinsVector::iterator begin = __bins.begin () + (int)__bins.size () / 2;
            typename LocalBinsVector::iterator end = __bins.end ();
            LocalBinsVector list;
            list.insert (list.end (), begin, end);
            return findBin (list, value);
          }
          return bin.index;
        }

      void
      addLinesToHistogram ()
      {
        int linesAdded = 0;
        for (size_t i = 0; i < lines.size (); i++)
        {
          LineSegment3D<PointT>& line = lines[i];
          int idx;

          //BinsVector bins;
          //findBin<4>(bins, 0.0f);

          idx = findBin<0> (matrix.bins, line.getCenter ()[compareIndex]);
          if (idx == -1) printf ("idx=-1, %d, ", line.getId ());
          if (idx > -1)
          {
            matrix[idx].push_back (line);
            linesAdded++;
          }
        }
        //printf ("addLinesToHistogram: numLines=%d, linesAdded=%d\n", (int)lines.size (), linesAdded);
      }

      void
      mergeLines2 (Bin<LineSegment3D<PointT> , 0>& bin)
      {
        if (bin.size () < 2) return;
        //printf ("merge: ");
        LineVector outLines;
        for (size_t i = 0; i < bin.size (); i++)
        {
          LineSegment3D<PointT>& line = bin[i];
          if (i == 0)
          {
            outLines.push_back (bin[i]);
          } else
          {
            bool added = false;
            //iterate on output lines and add our line to one of them if its close enough or create a new out line if not
            for (size_t j = 0; j < outLines.size (); j++)
            {
              LineSegment3D<PointT>& line2 = outLines[j];
              if (fabs (line.getCenter ()[0] - line2.getCenter ()[0]) < MAX_DIST_LINES
                  && fabs (line.getCenter ()[2] - line2.getCenter ()[2]) < MAX_DIST_LINES && line.getId ()
                  != line2.getId ())
              {
                added = true;
                line2 += line;
                //printf ("%d, %d", line2.getId (), line.getId ());
              }
            }
            //if no near line to our candidate line then add it separately to the output lines
            if (!added)
            {
              outLines.push_back (bin[i]);
            }
            //outLine += bin[i];
          }
        }
        bin.content.clear ();
        //bin.push_back (outLine);
        bin.content.assign (outLines.begin (), outLines.end ());
        printf ("\n");
      }

      void
      mergeLines (Bin<LineSegment3D<PointT> , 0>& bin)
      {
        if (bin.size () < 2) return;
        std::vector<int> ledger (bin.size (), 1);
        for (size_t i = 0; i < bin.size (); i++)
        {
          LineSegment3D<PointT>& line1 = bin[i];
          for (size_t j = i + 1; j < bin.size (); j++)
          {
            if (ledger[j] == -1) continue;
            LineSegment3D<PointT>& line2 = bin[j];
            if (line1.getId() == line2.getId()) continue;
            if (calcAngle3DDegree (line1.getNormal (), line2.getNormal ()) < MAX_ANGLE_BET_LINES
                && fabs (line1.getCenter ()[0] - line2.getCenter ()[0]) < MAX_DIST_LINES
                && fabs (line1.getCenter ()[2] - line2.getCenter ()[2]) < MAX_DIST_LINES)
            {
              line1 += line2;
              ledger[j] = -1;
              //printf("merge line %d, %d\n", line1.getId(), line2.getId());
            }
          }
        }
        LineVector outLines;
        for (size_t i = 0; i < bin.size (); i++)
        {
          if (ledger[i] > -1) outLines.push_back (bin[i]);
        }
        //printf ("merge=%d lines\n", (int)(bin.size () - outLines.size ()));
        bin.content = outLines;
      }

      void
      initHistogram ()
      {
        float min, max;
        findMinMax (min, max);
        float numHeightBins = ((max - min) / MAX_DIST_LINES) + 1;

        float rangeMin0, rangeMax0;

        matrix.bins.clear ();
        rangeMin0 = min, rangeMax0 = min + MAX_DIST_LINES;
        for (size_t l = 0; l < (size_t)numHeightBins; l++)
        {
          Bin<LineSegment3D<PointT> , 0> bin (l, rangeMin0, rangeMax0);
          rangeMin0 = rangeMax0;
          rangeMax0 += MAX_DIST_LINES;
          matrix.push_back (bin);
        }

      }
    public:

      LineSegment3DHistogram (int compareIndex = 2)
      {
        this->compareIndex = compareIndex;
        domNormal.Zero ();
      }

      void
      createHistogram ()
      {
        if (lines.size () > 0)
        {
          initHistogram ();
          addLinesToHistogram ();
        }
      }

      Eigen::Vector3f
      getDominantNormal ()
      {
        return domNormal;
      }

      inline void
      addLine (LineSegment3D<PointT>& line)
      {
        lines.push_back (line);
      }

      inline void
      addLines (LineVector& inLines)
      {
        lines.insert (lines.end (), inLines.begin (), inLines.end ());
      }

      void
      mergeLines ()
      {
        for (size_t l = 0; l < matrix.size (); l++)
        {
          Bin<LineSegment3D<PointT> , 0>& bin = matrix[l];
          mergeLines (bin);
        }
      }

      LineVector
      getLines ()
      {
        LineVector outLines;
        for (size_t l = 0; l < matrix.size (); l++)
        {
          Bin<LineSegment3D<PointT> , 0>& bin = matrix[l];
          outLines.insert (outLines.end (), bin.content.begin (), bin.content.end ());
        }
        return outLines;
      }

      void
      logContents ()
      {
        printf ("logging histogram contents\n");
        for (size_t l = 0; l < matrix.size (); l++)
        {
          Bin<LineSegment3D<PointT> , 0>& bin4 = matrix[l];
          if (bin4.size () > 0)
          {
            printf ("bin[%d]=", (int)l);
          }
          for (size_t m = 0; m < bin4.size (); m++)
          {
            printf ("%d, ", bin4[m].getId ());
          }
        }
      }

      void
      logHistogramStructure ()
      {
        printf ("logging histogram Structure\n");
        for (size_t l = 0; l < matrix.size (); l++)
        {
          Bin<LineSegment3D<PointT> , 0>& bin4 = matrix[l];
          printf ("Level1: %d, [%f,%f]\n", bin4.index, bin4.rangeMin, bin4.rangeMax);
        }
      }
    };
}

#define PCL_INSTANTIATE_LineSegment3DHistogram(T) template class PCL_EXPORTS pcl::LineSegment3DHistogram<T>;
#endif /* LINESEGMENT3DHISTOGRAM_H_ */
