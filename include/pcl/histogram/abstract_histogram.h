/*
 * histogram_.h
 *
 *  Created on: Jul 3, 2012
 *      Author: elmasry
 */

#ifndef ABSTRACT_HISTOGRAM__H_
#define ABSTRACT_HISTOGRAM__H_

#include <vector>

namespace pcl
{
  struct _IBin
  {
    float rangeMin, rangeMax;
    int index;
    _IBin ()
    {
    }
    _IBin (int index, float rangeMin, float rangeMax)
    {
      this->index = index;
      this->rangeMin = rangeMin;
      this->rangeMax = rangeMax;
    }

    _IBin (const _IBin& bin)
    {
      index = bin.index;
      rangeMin = bin.rangeMin;
      rangeMax = bin.rangeMax;
    }

    _IBin&
    operator= (const _IBin& bin)
    {
      index = bin.index;
      rangeMin = bin.rangeMin;
      rangeMax = bin.rangeMax;
      return (*this);
    }

    virtual size_t
    size () = 0;
  };

  template<typename T, unsigned nest>
    struct Bin : public _IBin
    {
      typedef Eigen::aligned_allocator<T> Alloc;
      std::vector<Bin<T, (nest - 1)> , Alloc> bins;

      Bin () :
        _IBin ()
      {
      }
      Bin (const Bin<T, nest>& bin) :
        _IBin (bin)
      {
        bins = bin.bins;
      }

      Bin (int index, float rangeMin, float rangeMax) :
        _IBin (index, rangeMin, rangeMax)
      {

      }
      Bin<T, nest>&
      operator= (const Bin<T, nest>& bin)
      {
        _IBin::operator= (bin);
        bins = bin.bins;
        return (*this);
      }
      Bin<T, (nest - 1)>&
      operator[] (const size_t index)
      {
        return bins[index];
      }

      void
      push_back (Bin<T, (nest - 1)>& bin)
      {
        bins.push_back (bin);
      }

      size_t
      size ()
      {
        return bins.size ();
      }

    };

  template<typename T>
    struct Bin<T, 0> : public _IBin
    {
      typedef Eigen::aligned_allocator<T> Alloc;
      std::vector<T, Alloc> content;

      Bin () :
        _IBin ()
      {
      }
      Bin (const Bin<T, 0>& bin) :
        _IBin (bin)
      {
        content = bin.content;
      }

      Bin (int index, float rangeMin, float rangeMax) :
        _IBin (index, rangeMin, rangeMax)
      {

      }
      Bin<T, 0>&
      operator= (const Bin<T, 0>& bin)
      {
        _IBin::operator= (bin);
        content = bin.content;
        return (*this);
      }

      T&
      operator[] (const size_t index)
      {
        return content[index];
      }

      void
      push_back (T& elem)
      {
        content.push_back (elem);
      }

      size_t
      size ()
      {
        return content.size ();
      }
    };

  template<typename T, unsigned dimension>
    class AbstractHistogram
    {
    protected:
      std::vector<Bin<T, dimension> > bins;

    };
}

#endif /* HISTOGRAM__H_ */
