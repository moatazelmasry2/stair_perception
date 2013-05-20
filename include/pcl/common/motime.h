/*
 * time.h
 *
 *  Created on: May 23, 2012
 *      Author: elmasry
 */

#ifndef TIME_H_
#define TIME_H_

#include <sys/time.h>
#include <unistd.h>

namespace pcl
{
    /**
     * returns time in milliseconds
     */
    inline long
    getTimeMs ()
    {
      struct timeval start;
      long mtime;

      gettimeofday (&start, NULL);

      mtime = ((start.tv_sec) * 1000 + start.tv_usec / 1000.0) + 0.5;

      return mtime;
    }
}
#endif /* MOTIME_H_ */
