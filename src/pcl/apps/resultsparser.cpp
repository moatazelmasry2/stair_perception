/*
 * resultsparser.cpp
 *
 *  Created on: Sep 16, 2012
 *      Author: elmasry
 */

#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <vector>
#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

std::vector<std::string> &
split (const std::string &s, char delim, std::vector<std::string> &elems)
{
  std::stringstream ss (s);
  std::string item;
  while (std::getline (ss, item, delim))
  {
    elems.push_back (item);
  }
  return elems;
}

std::vector<std::string>
split (const std::string &s, char delim)
{
  std::vector<std::string> elems;
  split (s, delim, elems);
  return elems;
}

int
main (int argc, char **argv)
{

  if (argc == 0) throw std::runtime_error ("no input file given");
  std::ifstream infile;

  infile.open (argv[1]);
  std::string line;
  int numLenghts = 0, numLDepths = 0, numRDepths = 0, numHeights = 0;
  double sumHeights = 0, sumLengths = 0, sumLDepths = 0, sumRDepth = 0;
  double l, d, h;
  while (getline (infile, line)) // To get you all the lines.
  {

    if (line.find ("Tread") != std::string::npos)
    {
      std::vector<std::string> tokens = split (line, ' ');
      if (tokens.size () < 4) throw std::runtime_error ("number tokens for Tread < 4");
      if ( (l = atof (tokens[1].c_str () ) ) < 1.5)
      {
        sumLengths += l;
        numLenghts++;
      }
      if ( ( d = atof (tokens[2].c_str () ) ) < 0.4)
      {
        sumLDepths += d;
        numLDepths++;
      }

      if ( ( d = atof (tokens[3].c_str () ) ) < 0.4)
      {
        sumRDepth += d;
        numRDepths++;
      }

    }

    if (line.find ("Riser") != std::string::npos)
    {
      std::vector<std::string> tokens = split (line, ' ');
      if (tokens.size () < 3) throw std::runtime_error ("number tokens for Riser < 3");
      if ((l = atof (tokens[1].c_str () ) ) < 1.5)
      {
        sumLengths += l;
        numLenghts++;
      }

      if ((h = atof (tokens[2].c_str () ) )< 0.4)
      {
        sumHeights += h;
        numHeights++;
      }
    }
  }
  infile.close ();

  float avgLength = sumLengths / numLenghts;
  printf ("average Length=%f, leftDepth=%f, rightDepth=%f, height=%f\n", avgLength, sumLDepths / numLDepths,
          sumRDepth / numRDepths, sumHeights / numHeights);

}
