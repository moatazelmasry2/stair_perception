/*
 * tread.h
 *
 *  Created on: Aug 16, 2012
 *      Author: elmasry
 */

#ifndef TREAD_H_
#define TREAD_H_

#include "pcl/types/plane3d.h"

namespace pcl
{
  template<typename PointT>
    class Tread : public Plane3D<PointT>
    {

    protected:

    public:

      typedef typename PointCloud<PointT>::ConstPtr PointCloudConstPtr;
      typedef boost::shared_ptr<Tread<PointT> > Ptr;

      using pcl::Plane3D<PointT>::normal;
      using pcl::Plane3D<PointT>::center;
      using pcl::Plane3D<PointT>::length;
      using pcl::Plane3D<PointT>::lDepth;
      using pcl::Plane3D<PointT>::rDepth;

      const static float minLandingLength = 0.5f;
      const static float minLandingDepth = 0.5f;

    protected:

    protected:

    public:
      Tread () :
        Plane3D<PointT> ()
      {
      }
      Tread (int id) :
        Plane3D<PointT> (id)
      {

      }

      Tread (const Tread<PointT>& tread) :
        Plane3D<PointT> ((Plane3D<PointT> )tread)
      {
      }

      Tread<PointT>&
      operator= (const Tread<PointT>& tread)
      {
        Plane3D<PointT>::operator= (tread);
        return (*this);
      }

      Tread<PointT>&
      operator+= (const Tread<PointT>& tread)
      {

        Plane3D<PointT> * p2 = (Plane3D<PointT>*)&tread;
        Plane3D<PointT>::operator+= (*p2);
        return (*this);
      }

      inline Ptr
      makeShared ()
      {
        return Ptr (new Tread<PointT> (*this));
      }

      inline bool
      isLanding () const
      {
        if (this->length >= minLandingLength && (this->lDepth >= minLandingDepth
            || this->rDepth >= minLandingDepth))
        {
          return true;
        }
        return false;
      }

      /*inline bool
       isMinSize ()
       {
       //printf("lDepth=%f, rDepth=%f, minTD=%f,  l=%f, minL=%f\n", this->lDepth, this->rDepth,  Tread<PointT>::minTreadDepth,  this->length,  Tread<PointT>::minStepLength);
       if (this->lDepth >= Tread<PointT>::minTreadDepth && this->rDepth >= Tread<PointT>::minTreadDepth && this->length >= Tread<PointT>::minStepLength)
       {
       return true;
       }
       return false;
       }*/
    };
}

#define PCL_INSTANTIATE_Tread(T) template class PCL_EXPORTS pcl::Tread<T>;
#endif /* TREAD_H_ */
