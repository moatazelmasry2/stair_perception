/*
 * boundingbox.h
 *
 *  Created on: Aug 23, 2012
 *      Author: elmasry
 */

#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_

namespace pcl
{
  template<typename PointT>
    class BoundingBox
    {
    public:

      PointT topLeft, topRight, bottomLeft, bottomRight;

      BoundingBox(){}
      BoundingBox (const BoundingBox<PointT>& bbox)
      {
        this->topLeft = bbox.topLeft;
        this->topRight = bbox.topRight;
        this->bottomLeft = bbox.bottomLeft;
        this->bottomRight = bbox.bottomRight;
      }

      BoundingBox<PointT>&
      operator= (const BoundingBox<PointT>& bbox)
      {
        this->topLeft = bbox.topLeft;
        this->topRight = bbox.topRight;
        this->bottomLeft = bbox.bottomLeft;
        this->bottomRight = bbox.bottomRight;
        return (*this);
      }
    };
}

#define PCL_INSTANTIATE_BoundingBox(T) template class PCL_EXPORTS pcl::BoundingBox<T>;
#endif /* BOUNDINGBOX_H_ */
