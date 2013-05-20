/*
 * riser.h
 *
 *  Created on: Aug 16, 2012
 *      Author: elmasry
 */

#ifndef RISER_H_
#define RISER_H_

#include "pcl/types/plane3d.h"

namespace pcl
{
  template<typename PointT>
    class Riser : public Plane3D<PointT>
    {

    protected:

    public:

      typedef boost::shared_ptr<Riser<PointT> > Ptr;

      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      using pcl::Plane3D<PointT>::normal;
      using pcl::Plane3D<PointT>::center;
      using pcl::Plane3D<PointT>::length;
      using pcl::Plane3D<PointT>::height;

      Riser ()
      {
      }
      Riser (int id) :
        Plane3D<PointT> (id)
      {

      }

      Riser (const Riser<PointT>& riser) :
        Plane3D<PointT> ((Plane3D<PointT> )riser)
      {
      }

      Riser<PointT>&
      operator= (const Riser<PointT>& riser)
      {
        Plane3D<PointT>::operator= (riser);
        return (*this);
      }

      Riser<PointT>&
      operator+= (const Riser<PointT>& riser)
      {
        Plane3D<PointT> * p2 = (Plane3D<PointT>*)&riser;
        Plane3D<PointT>::operator+= (*p2);
        return (*this);
      }

      inline Ptr
      makeShared ()
      {
        return Ptr (new Riser<PointT> (*this));
      }

      /*inline bool
       isMinSize ()
       {
       //printf("l=%f, minL=%f, h=%f, minh=%f\n", this->length,  Riser<PointT>::minStepLength,  this->height,  Riser<PointT>::minRiserHeight);
       if (this->length >= Riser<PointT>::minStepLength && this->height >= Riser<PointT>::minRiserHeight)
       {
       return true;
       }
       return false;
       }*/
    };
}

#define PCL_INSTANTIATE_Riser(T) template class PCL_EXPORTS pcl::Riser<T>;
#endif /* RISER_H_ */
