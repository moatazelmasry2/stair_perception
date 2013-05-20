/*
 * linesegment2dhistogram.h
 *
 *  Created on: Apr 19, 2012
 *      Author: elmasry
 */

#ifndef LINESEGMENT2D_HISTOGRAM_H_
#define LINESEGMENT2D_HISTOGRAM_H_

#include <vector>
#include <cmath>

#include "pcl/opencv/linesegment2d.h"
#include <pcl/common/math.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace pcl
{

  class ColumnBin
  {
    //typedef Eigen::aligned_allocator Alloc;
    typedef std::vector<LineSegment2D> LineVector;

  public:

    int id;
    float rangeMin, rangeMax;
    LineVector lines;

    ColumnBin ()
    {
    }
    ColumnBin (int id, float min, float max)
    {
      this->id = id;
      rangeMin = min;
      rangeMax = max;
    }

    ColumnBin (const ColumnBin& bin)
    {
      id = bin.id;
      rangeMin = bin.rangeMin;
      rangeMax = bin.rangeMax;
      lines = bin.lines;
    }

    ColumnBin&
    operator= (const ColumnBin& bin)
    {
      id = bin.id;
      rangeMin = bin.rangeMin;
      rangeMax = bin.rangeMax;
      lines = bin.lines;
      return (*this);
    }

    void
    addLine (LineSegment2D line)
    {
      lines.push_back (line);
    }
  };

  class RowBin
  {
    //typedef Eigen::aligned_allocator Alloc;
    typedef std::vector<ColumnBin> ColumnBins;

  public:

    int id;
    float rangeMin, rangeMax;
    ColumnBins colBins;

    RowBin ()
    {
    }
    RowBin (int id, float min, float max)
    {
      this->id = id;
      rangeMin = min;
      rangeMax = max;
    }

    RowBin (const RowBin& bin)
    {
      id = bin.id;
      rangeMin = bin.rangeMin;
      rangeMax = bin.rangeMax;
      colBins = bin.colBins;
    }

    RowBin&
    operator= (const RowBin& bin)
    {
      id = bin.id;
      rangeMin = bin.rangeMin;
      rangeMax = bin.rangeMax;
      colBins = bin.colBins;
      return (*this);
    }
  };

  class LineSegment2DHistogram
  {

#define METHOD_STANDARD 1
#define METHOD_P        2
    typedef std::vector<ColumnBin> ColumnBins;
    typedef std::vector<RowBin> BinsMatrix;
    typedef std::vector<LineSegment2D> LineVector;
  protected:

    //BinsVector bins;
    LineVector lines;
    float minR, maxR, minTheta, maxTheta;
    BinsMatrix matrix, oldMatrix;
    /** dominant angle in degrees*/
    float dominantAngle;
    float maxY, maxX;

    /**
     * extracts min and max z of lines (camera coordinates)
     */
    void
    getMinMaxR (float& minR, float& maxR)
    {
      for (size_t i = 0; i < lines.size (); i++)
      {
        LineSegment2D& line = lines[i];
        if (line.getR () > maxR)
        {
          maxR = line.getR ();
        }

        if (line.getR () < minR)
        {
          minR = line.getR ();
        }
      }
    }

    void
    logBins ()
    {
      std::cout << "minRho=" << minR << ", maxRho=" << maxR << ", minTheta=" << minTheta << ", maxTheta=" << maxTheta
          << std::endl;
      ColumnBins tmpBins = matrix[0].colBins;
      std::cout << "logging columns" << std::endl;
      for (size_t i = 0; i < tmpBins.size (); i++)
      {
        ColumnBin& tmpBin = tmpBins[i];
        std::cout << i << "min=" << tmpBin.rangeMin << ", max=" << tmpBin.rangeMax << std::endl;
      }

      std::cout << "-----------------------" << std::endl << "Logging rows" << std::endl;
      for (size_t i = 0; i < matrix.size (); i++)
      {
        RowBin& tmpbin = matrix[i];
        std::cout << i << "min=" << tmpbin.rangeMin << ", max=" << tmpbin.rangeMax << std::endl;
      }
      std::cout << "-----------------" << std::endl;
    }

    void
    initBins ()
    {
      matrix.clear ();
      //float step = 10.0f;
      float tmpMin = minTheta, tmpMax = tmpMin + THETA_STEP;
      int count = 0;
      ColumnBins colBins;
      int numCols = ceil ((maxTheta - minTheta) / THETA_STEP);
      numCols = numCols > 0? numCols : 1;
      for (int i = 0; i < numCols; i++)
      {
        ColumnBin colBin (count++, tmpMin, tmpMax);
        colBins.push_back (colBin);
        tmpMin = tmpMax;
        tmpMax += THETA_STEP;
      }

      count = 0;
      int numRows = ceil ((maxR - minR) / RHO_STEP);
      numRows = numRows > 0? numRows : 1;
      tmpMin = minR, tmpMax = tmpMin + RHO_STEP;
      for (int i = 0; i < numRows; i++)
      {
        RowBin rowBin (count++, tmpMin, tmpMax);
        int count2 = 0;
        for (size_t j = 0; j < colBins.size (); j++)
        {
          ColumnBin colBin (count2++, colBins[j].rangeMin, colBins[j].rangeMax);
          rowBin.colBins.push_back (colBin);
        }
        //rowBin.colBins.assign (colBins.begin (), colBins.end ());
        matrix.push_back (rowBin);
        tmpMin = tmpMax;
        tmpMax += RHO_STEP;
      }
    }

    /**
     * Find the gith bin for the rows
     */
    int
    findBin (BinsMatrix __bins, float value)
    {
      if (__bins.size () == 1)
      {
        RowBin b = __bins[0];
        if (value >= b.rangeMin && value <= b.rangeMax)
        {
          return b.id;
        }
        else
        {
          return -1;
        }
      }

      if (__bins.size () < 1)
      {
        std::cerr << "2DLineSegmentHistogram bins.size=0. This should not happen.val=" << value << ", minR="<< this->minR << ", maxR="<< maxR;
        std::cerr << ", numLines="<< lines.size() << std::endl;
        return -1;
      }

      //printf("bins.size=%d\n", (int)__bins.size ());
      RowBin bin = __bins[(int)__bins.size () / 2];
      if (value < bin.rangeMin)
      {
        BinsMatrix::iterator begin = __bins.begin ();
        BinsMatrix::iterator end = __bins.begin () + ((int)__bins.size () / 2);
        BinsMatrix list;
        list.assign (begin, end);
        if (list.size () == 0)
        {
          return -1;
        }
        return findBin (list, value);
      }
      else if (value > bin.rangeMax)
      {
        BinsMatrix::iterator begin = __bins.begin () + ( (int)__bins.size () / 2 );
        BinsMatrix::iterator end = __bins.end ();
        BinsMatrix list;
        list.insert (list.end (), begin, end);
        if (list.size () == 0)
        {
          return -1;
        }
        return findBin (list, value);
      }
      else
      {
        return bin.id;
      }
    }

    /**
     * find bin for the column
     */
    int
    findBin (ColumnBins __bins, float value)
    {
      ColumnBin dummy;
      if ((int)__bins.size () == 1)
      {
        ColumnBin& b = __bins[0];
        if (value >= b.rangeMin && value <= b.rangeMax)
        {
          return b.id;
        }
        else
        {
          return -1;
        }
      }

      ColumnBin& bin = __bins[(int)__bins.size () / 2];
      if (value < bin.rangeMin)
      {
        ColumnBins::iterator begin = __bins.begin ();
        ColumnBins::iterator end = __bins.begin () + (int)__bins.size () / 2;
        ColumnBins list;
        list.assign (begin, end);
        if (list.size () == 0)
        {
          return -1;
        }
        return findBin (list, value);
      }
      else if (value > bin.rangeMax)
      {
        ColumnBins::iterator begin = __bins.begin () + (int)__bins.size () / 2;
        ColumnBins::iterator end = __bins.end ();
        ColumnBins list;
        list.insert (list.end (), begin, end);
        if (list.size () == 0)
        {
          return -1;
        }
        return findBin (list, value);
      }
      else
      {
        return bin.id;
      }
      /*else
       {
       return -1;
       }*/
    }

    void
    addLinesToHistogram ()
    {
      for (size_t i = 0; i < lines.size (); i++)
      {
        LineSegment2D& line = lines[i];

        int rowIdx = findBin (matrix, line.getR ());
        if (rowIdx >= 0)
        {

          int colIdx = findBin (matrix[rowIdx].colBins, line.getThetaDegree ());
          if (colIdx >= 0)
          {
            ColumnBin& bin = matrix[rowIdx].colBins[colIdx];
            bin.addLine (line);
          }
        }
      }
    }

    int
    mergeLinesP (int row, int column)
    {
      ColumnBin& colBin = matrix[row].colBins[column];
      return mergeLinesP (colBin);
    }

    /**
     * return number of merges
     */
    int
    mergeLinesP (ColumnBin& bin)
    {
      LineVector& lines = bin.lines;
      std::vector<int> ledger ((int)lines.size (), 1);
      int merges = 0;
      for (size_t i = 0; i < lines.size (); i++)
      {
        if (ledger[i] < 0)
        {
          continue;
        }
        LineSegment2D& line1 = lines[i];
        for (size_t j = i + 1; j < lines.size (); j++)
        {
          if (ledger[j] < 0)
          {
            continue;
          }
          LineSegment2D& line2 = lines[j];
          //ydiff is used to compare parallel and close lines that could me bermegd together
          //float yDiff = fabs ((line1.getCentroid ()[1] - line2.getCentroid ()[1]));
          //float xDiff = fabs ((line1.getCentroid ()[0] - line2.getCentroid ()[0]));
          //float thetaDegree = line1.getThetaDegree ();
          //xDist is used to make sure that when combining two horizontal lines, that lie close to each other

          //float xDist = std::min (fabs (line1.getLineModel ()[0] - line2.getLineModel ()[2]),
          //                        fabs (line1.getLineModel ()[2] - line2.getLineModel ()[0]));

          /*if ((yDiff <= maxY && thetaDegree > 45 && thetaDegree < 135 && xDist < maxX) || (yDiff <= maxY && thetaDegree
              > 45 && thetaDegree < 135 && xDist < maxX))*/
          //if ((yDiff <= maxY && xDist < maxX))
          {
            merges++;
            //for the newly constructed line:
            //if theta < 45, take min max x and average y
            //else take min max y and average x
            //theta is the average of thetas of the two lines
            float avgTheta = (line1.getThetaDegree () + line2.getThetaDegree ()) / 2;
            cv::Vec4i model1 = line1.getLineModel (), model2 = line2.getLineModel ();
            cv::Vec4i model;

            //Horizontal line
            float ang = fabs (line2.getTheta () - dominantAngle) < fabs (line1.getTheta () - dominantAngle)
                ? line2.getTheta () : line1.getTheta ();
            if (avgTheta > 45 && avgTheta < 135)
            {

              model[0] = std::min (std::min (model1[0], model2[0]), std::min (model1[2], model2[2]));
              model[2] = std::max (std::max (model1[0], model2[0]), std::max (model1[2], model2[2]));
              if (fabs (pcl::radianToAngle (dominantAngle) - line1.getThetaDegree ()) < 20
                  && fabs (pcl::radianToAngle (dominantAngle) - line2.getThetaDegree ()) < 20)
              {//make the line aligned to the dominant orientation
                model[1] = LineSegment2D::findY (model[0], ang, (line1.getR () + line2.getR ()) / 2);
                model[3] = LineSegment2D::findY (model[2], ang, (line1.getR () + line2.getR ()) / 2);
              }
              else
              {
                model[1] = (model1[1] + model2[1]) / 2;
                model[3] = (model1[3] + model2[3]) / 2;
              }

            }
            else
            {
              model[1] = std::min (std::min (model1[1], model1[3]), std::min (model2[1], model2[3]));
              model[3] = std::max (std::max (model1[1], model1[3]), std::max (model2[1], model2[3]));
              if (fabs (pcl::radianToAngle (dominantAngle) - line1.getThetaDegree ()) < 20
                  && fabs (pcl::radianToAngle (dominantAngle) - line2.getThetaDegree ()) < 20)
              {//make the line aligned to the dominant orientation
                model[0] = LineSegment2D::findX (model[1], ang, (line1.getR () + line2.getR ()) / 2);
                model[2] = LineSegment2D::findX (model[3], ang, (line1.getR () + line2.getR ()) / 2);
              }
              else
              {
                model[0] = (model1[0] + model2[0]) / 2;
                model[2] = (model1[2] + model2[2]) / 2;
              }
            }
            line1.setLineModel (model);
            ledger[j] = -1;
          }
        }
      }
      int befMerge = lines.size ();
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
      int afterMerge = lines.size ();
      return befMerge - afterMerge;
    }

    void
    mergeLines2 (ColumnBin& bin)
    {
      //float maxDist = 10.0f, maxAngle = 10.0f;
      LineVector& lines = bin.lines;
      float tmpRho = 0, tmpTheta = 0;
      if (lines.size () == 0)
      {
        return;
      }
      for (size_t i = 0; i < lines.size (); i++)
      {
        LineSegment2D& line1 = lines[i];
        tmpRho += line1.getR ();
        tmpTheta += line1.getTheta ();
      }

      tmpRho /= lines.size ();
      tmpTheta /= lines.size ();
      LineSegment2D outLine;
      outLine.setLineModel (tmpRho, tmpTheta);
      lines.clear ();
      lines.push_back (outLine);
    }

    void
    mergeLinesP2 (ColumnBin& bin)
    {
      //float maxDist = 10.0f, maxAngle = 10.0f;
      LineVector& lines = bin.lines;
      //float tmpRho = 0, tmpTheta = 0;
      if (lines.size () == 0)
      {
        return;
      }

      std::vector<int> ledger;
      ledger.resize (lines.size (), 1);

      for (size_t i = 0; i < lines.size (); i++)
      {
        if (ledger[i] < 0)
          continue;
        LineSegment2D& line1 = lines[i];
        for (size_t j = i + 1; j < lines.size (); j++)
        {
          if (ledger[j] < 0)
            continue;
          LineSegment2D& line2 = lines[i];
          if (fabs (line1.getCentroid ()[1] - line2.getCentroid ()[1]) < 8)
          {
            if (line1.getThetaDegree () > 45 && line1.getThetaDegree () < 135)
            {
              //cv::Vec4i
            }
            else
            {

            }
          }
        }

      }
    }

    void
    calculateRhoWeights ()
    {
      //if bins size=0, return;
      if (matrix.size () == 0)
        return;
      std::vector<int> angles;
      angles.resize (matrix[0].colBins.size (), 0);
      for (size_t i = 0; i < matrix.size (); i++)
      {
        ColumnBins& colBins = matrix[i].colBins;

        for (size_t j = 0; j < colBins.size (); j++)
        {
          LineVector& lines = colBins[j].lines;
          for (size_t k = 0; k < lines.size (); k++)
          {
            LineSegment2D& line = lines[k];
            calculateRhoWeight (matrix[i], line);
          }
        }
      }
    }

    /**
     * calculate weights of all lines in a matrix entry
     */
    void
    calculateWeights (int row, int col)
    {
      //if bins size=0, return;
      if (matrix.size () == 0)
        return;
      std::vector<int> angles;
      angles.resize (matrix[0].colBins.size (), 0);
      RowBin& rowBin = matrix[row];
      ColumnBin& colBin = matrix[row].colBins[col];

      LineVector& lines = colBin.lines;
      for (size_t k = 0; k < lines.size (); k++)
      {
        LineSegment2D& line = lines[k];
        calculateRhoWeight (rowBin, line);
        calculateThetaWeight (colBin, line);
      }
    }

    inline void
    calculateThetaWeight (const ColumnBin& bin, LineSegment2D& line)
    {
      line.thetaWeight = pcl::translateNumberRange (line.getThetaDegree (), bin.rangeMin, bin.rangeMax, 0.0f, 1.0f);
    }

    inline void
    calculateRhoWeight (const RowBin& bin, LineSegment2D& line)
    {
      line.rhoWeight = pcl::translateNumberRange (line.getR (), bin.rangeMin, bin.rangeMax, 0.0f, 1.0f);
    }

    void
    calculateThetaWeights ()
    {
      //if bins size=0, return;
      if (matrix.size () == 0)
        return;
      std::vector<int> angles;
      angles.resize (matrix[0].colBins.size (), 0);
      for (size_t i = 0; i < matrix.size (); i++)
      {
        ColumnBins& colBins = matrix[i].colBins;

        for (size_t j = 0; j < colBins.size (); j++)
        {
          LineVector& lines = colBins[j].lines;
          for (size_t k = 0; k < lines.size (); k++)
          {
            LineSegment2D& line = lines[k];
            calculateThetaWeight (colBins[j], line);
          }
        }
      }
    }
    void
    calculateOrientations ()
    {
      //if bins size=0, return;
      if (matrix.size () == 0)
        return;
      std::vector<int> angles;
      //create a histogram with 90 bins, each holds 2 degrees
      angles.resize (90, 0);
      for (size_t i = 0; i < lines.size (); i++)
      {
        LineSegment2D& line = lines[i];
        float deg = line.getThetaDegree ();
        //add +1 to the corresponding bin
        int index = (int)(((int)deg) / 2);
        angles[index] = angles[index] + 1;
      }
      int idx = 0;
      int dom = 0;
      for (size_t i = 0; i < angles.size (); i++)
      {
        if (angles[i] > dom)
        {
          dom = angles[i];
          idx = i;
        }
      }

      dominantAngle = pcl::degreeToRadian (idx * 2 + 1);
    }

    /**
     * removes lines whose orientations are far than 10 degrees from the dominant orientation
     */
    void
    filterInadequateOrientations ()
    {
      //if bins size=0, return;
      if (matrix.size () == 0)
        return;
      LineVector tmpLines;
      float domtheta = pcl::radianToAngle (dominantAngle);
      for (size_t i = 0; i < lines.size (); i++)
      {
        LineSegment2D& tmpLine = lines[i];
        if (fabs (domtheta - tmpLine.getThetaDegree ()) < 10)
        {
          tmpLines.push_back (tmpLine);
        }
      }
      lines.clear ();
      lines.assign (tmpLines.begin (), tmpLines.end ());
    }

    int
    migrateNeighboringLines (int row1, int col1, int row2, int col2, float tLowerBound, float tUpperBound,
                           float rLowerBound, float rUpperBound)
    {
      ColumnBin& bin1 = matrix[row1].colBins[col1];
      ColumnBin& bin2 = matrix[row2].colBins[col2];
      std::vector<int> ledger (bin1.lines.size (), 1);

      bool numMigrations = 0;
      for (size_t i = 0; i < bin2.lines.size (); i++)
      {
        LineSegment2D& line2 = bin2.lines[i];
        for (size_t j = 0; j < bin1.lines.size (); j++)
        {
          if (ledger[j] < 0)
            continue;
          LineSegment2D& line1 = bin1.lines[j];
          bool b1, b2, b3, b4;

          //if same row don't check rhoWeight. rho doesn't vary on the same row
          if (row1 == row2)
          {
            b1 = b2 = true;
          }
          else
          {
            b1 = (line1.rhoWeight > rUpperBound);
            b2 = (line2.rhoWeight < rLowerBound);
          }

          if (col1 == col2)
          {
            b3 = b4 = true;
          }
          else
          {
            b3 = line1.thetaWeight > tUpperBound;
            b4 = line2.thetaWeight < tLowerBound;
          }

          //if (line1.rhoWeight > rUpperBound && line2.rhoWeight < rLowerBound && line1.thetaWeight > tUpperBound
          //    && line2.thetaWeight < tLowerBound)
          if (b1 && b2 && b3 && b4)
          {
            //line1.rhoWeight = pcl::translateNumberRange(line1.getR(), bin2.)
            bin2.lines.push_back (line1);
            ledger[j] = -1;
            numMigrations++;
          }
        }
        LineVector tmpLines;
        for (size_t i = 0; i < bin1.lines.size(); i++) {
          if (ledger[i] > 0) {
            tmpLines.push_back(bin1.lines[i]);
          }
        }
        bin1.lines.clear();
        bin1.lines.assign(tmpLines.begin(), tmpLines.end());
      }

      if (numMigrations > 0)
      {
        calculateWeights (row2, col2);
        return numMigrations;
      }
      return 0;
    }

  public:

    float THETA_STEP;
    float RHO_STEP;

    LineSegment2DHistogram ()
    {
      THETA_STEP = 8;
      RHO_STEP = 15;
      maxY = 8.0f;
      maxX = 100.0f;
    }
    void
    logMatrix ()
    {
      for (size_t i = 0; i < matrix.size (); i++)
      {
        ColumnBins colBins = matrix[i].colBins;
        for (size_t j = 0; j < colBins.size (); j++)
        {
          ColumnBin& bin = colBins[j];
          std::cout << bin.lines.size () << " ";
        }
        std::cout << std::endl;
      }
      std::cout << "--------------------------" << std::endl;

      for (size_t i = 0; i < matrix.size (); i++)
      {
        ColumnBins colBins = matrix[i].colBins;
        for (size_t j = 0; j < colBins.size (); j++)
        {
          ColumnBin& bin = colBins[j];
          if (bin.lines.size () > 0)
          {
            std::cout << "bin:" << i << "," << j << ", minR =" << bin.rangeMin << ", maxR=" << bin.rangeMax
                << std::endl;
          }
          for (size_t k = 0; k < bin.lines.size (); k++)
          {
            LineSegment2D line = bin.lines[k];
            std::cout << "line" << line.getId () << "rho=" << line.getR () << ", theta=" << line.getThetaDegree ()
                << ", y=" << line.getCentroid ()[1] << std::endl;
          }
        }
      }
      std::cout << "--------------------------" << std::endl;
    }
    /**
     * See if two lines in the same column have close rho,
     * then move line from the lower bin to the upper bin
     */
    /*void
     migrateNeighboringBins ()
     {
     float lowerBound = 0.5, upperBound = 0.5;
     int numCols = matrix[0].colBins.size ();
     int numMigrations = 0;
     //iterate through columns,each cycle iterate through rows
     for (int i = 0; i < numCols; i++)
     {
     for (size_t j = 0; j < matrix.size (); j++)
     {
     //idnicate whether two bins have neighboring lines
     bool neighborsBin1 = false, neighborsBin2;
     ColumnBin& bin1 = matrix[j].colBins[i];
     for (size_t k = 0; k < bin1.lines.size (); k++)
     {
     LineSegment2D& line = bin1.lines[k];
     if (line.rhoWeight > upperBound)
     {
     neighborsBin1 = true;
     break;
     }
     }//end for loop on bin1.lines
     if (j + 1 < matrix.size () && neighborsBin1)
     {
     ColumnBin& bin2 = matrix[j + 1].colBins[i];
     std::vector<int> ledger (bin2.lines.size (), 1);
     bool moveLine = false;
     for (size_t k = 0; k < bin2.lines.size (); k++)
     {
     LineSegment2D& line = bin2.lines[k];
     if (line.rhoWeight < lowerBound)
     {
     moveLine = true;
     ledger[k] = -1;
     bin1.lines.push_back (line);
     numMigrations++;
     }
     }
     if (moveLine)
     {
     LineVector tmpLines;
     for (size_t k = 0; k < bin2.lines.size (); k++)
     {
     if (ledger[k] >= 0)
     {
     tmpLines.push_back (bin2.lines[k]);
     }
     }
     bin2.lines.clear ();
     bin2.lines.assign (tmpLines.begin (), tmpLines.end ());
     }
     }
     }//end for loop on rows
     }//end for loop on columns
     std::cout << "nummigrations=" << numMigrations << std::endl;
     }*/

    void
    migrateNeighboringBins2 ()
    {
      oldMatrix = matrix;
      //if bins size=0, return;
      if (matrix.size () == 0)
        return;
      //these are used only for debug
      int migrations, merges;
      //theta lower and upper bound
      float tlowerBound = 0.2, tUpperBound = 0.2;
      //rho lower and upper bound
      float rlowerBound = 0.2, rUpperBound = 0.2;
      migrations = 0;
      merges = 0;
      for (size_t i = 0; i < matrix.size (); i++)
      {
        RowBin& rowBin = matrix[i];
        for (size_t j = 0; j < rowBin.colBins.size (); j++)
        {
          ColumnBin& colBin = rowBin.colBins[j];
          std::vector<int> ledger (colBin.lines.size (), 1);

          for (size_t k = 0; k < colBin.lines.size (); k++)
          {
            //if i+1, j+1
            if ((i + 1) < matrix.size () && (j + 1) < matrix[i + 1].colBins.size ())
            {
              int tMerges = migrateNeighboringLines (i, j, i + 1, j + 1, tlowerBound, tUpperBound, rlowerBound,
                                                   rUpperBound);
              if (tMerges > 0)
              {
                mergeLinesP (i + 1, j + 1);
                migrations += tMerges;
              }

            }
            //case row=i+1, col=j
            if ((i + 1) < matrix.size ())
            {
              int tMerges = migrateNeighboringLines (i, j, i + 1, j, tlowerBound, tUpperBound, rlowerBound, rUpperBound);
              if (tMerges > 0)
              {
                merges += mergeLinesP (i + 1, j);
                migrations += tMerges;
              }
            }

            //case row=i, col=j+1
            if ((j + 1) < matrix[i].colBins.size ())
            {
              int tMerges = migrateNeighboringLines (i, j, i, j + 1, tlowerBound, tUpperBound, rlowerBound, rUpperBound);
              if (tMerges > 0)
              {
                mergeLinesP (i, j + 1);
                migrations += tMerges;
              }
            }
          }//end for loop on lines
        }//end for loop columns
      }//end for loop on rows
    }

    /**
     * See if two lines in the same column have close theta,
     * then move line from the lower bin to the upper bin
     */
    /*void
     migrateNeighboringBinsTheta ()
     {
     float lowerBound = 0.3, upperBound = 0.7;
     int numCols = matrix[0].colBins.size ();
     int numMigrations = 0;
     //iterate through columns,each cycle iterate through rows
     for (size_t i = 0; i < matrix.size (); i++)
     {
     for (size_t j = 0; j < matrix[i].colBins.size (); j++)
     {
     //idnicate whether two bins have neighboring lines
     bool neighborsBin1 = false, neighborsBin2;
     ColumnBin& bin1 = matrix[i].colBins[j];
     for (size_t k = 0; k < bin1.lines.size (); k++)
     {
     LineSegment2D& line = bin1.lines[k];
     if (line.thetaWeight > upperBound)
     {
     neighborsBin1 = true;
     break;
     }
     if (j + 1 < matrix[i].colBins.size () && neighborsBin1)
     {
     ColumnBin& bin2 = matrix[i].colBins[j + 1];
     std::vector<int> ledger (bin2.lines.size (), 1);
     bool moveLine = false;
     for (size_t k = 0; k < bin2.lines.size (); k++)
     {
     LineSegment2D& line = bin2.lines[k];
     if (line.thetaWeight < lowerBound)
     {
     moveLine = true;
     ledger[k] = -1;
     bin1.lines.push_back (line);
     numMigrations++;
     }
     }
     if (moveLine)
     {
     LineVector tmpLines;
     for (size_t k = 0; k < bin2.lines.size (); k++)
     {
     if (ledger[k] >= 0)
     {
     tmpLines.push_back (bin2.lines[k]);
     }
     }
     bin2.lines.clear ();
     bin2.lines.assign (tmpLines.begin (), tmpLines.end ());
     }
     }
     }//end for loop on bin1.lines

     }//end for loop on rows
     }//end for loop on columns
     std::cout << "nummigrations=" << numMigrations << std::endl;
     }*/

    BinsMatrix
    getMatrix ()
    {
      return matrix;
    }

    BinsMatrix
    getOldMatrix ()
    {
      return oldMatrix;
    }

    void
    addLine (LineSegment2D line)
    {
      lines.push_back (line);
    }

    void
    addLines (LineVector lines)
    {
      this->lines.insert (this->lines.end (), lines.begin (), lines.end ());
    }

    void
    init ()
    {
      matrix.clear ();
      minR = 1000, maxR = -1000;
      minTheta = 0, maxTheta = 180;

      getMinMaxR (minR, maxR);
      initBins ();
      calculateOrientations ();
      filterInadequateOrientations ();
      addLinesToHistogram ();
      //logMatrix ();
      oldMatrix = matrix;

      calculateRhoWeights ();
      calculateThetaWeights ();
    }
    void
    calculate ()
    {
      //if bins size=0, return;
      if (matrix.size () == 0)
        return;
      for (size_t i = 0; i < matrix.size (); i++)
      {
        ColumnBins& colBins = matrix[i].colBins;
        for (size_t j = 0; j < colBins.size (); j++)
        {
          ColumnBin& bin = colBins[j];
          mergeLinesP (bin);
        }
      }
    }

    void
    getLines (LineVector& outLines)
    {
      //lines.clear ();
      for (size_t i = 0; i < matrix.size (); i++)
      {
        ColumnBins colBins = matrix[i].colBins;
        for (size_t j = 0; j < colBins.size (); j++)
        {
          LineVector ls = colBins[j].lines;
          outLines.insert (outLines.end (), ls.begin (), ls.end ());
        }

      }
      //return outLines;
    }

    float
    getDominantOrientation ()
    {
      return dominantAngle;
    }
  };
}

#endif /* HISTOGRAM_H_ */
