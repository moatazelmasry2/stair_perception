#ifndef FHG_PCL_TYPES_H_
#define FHG_PCL_TYPES_H_

#include <pcl/pcl_base.h>

namespace pcl
{

  template<typename PointT>
    class LineSegment
    {

    private:
      PointT p1, p2;
      bool p1Set, p2Set;

    public:
      LineSegment ()
      {
        p1Set = false;
        p2Set = false;
      }

      LineSegment(const LineSegment& l) {
        p1 = l.p1;
        p2 = l.p2;
        p1Set = l.p1Set;
        p2Set = l.p2Set;
      }

      LineSegment (PointT p1, PointT p2)
      {
        setP1(p1);
        setP2(p2);
      }

      void
      setP1 (PointT p1)
      {
        p1Set = true;
        this->p1 = p1;
      }

      PointT
      getP1 ()
      {
        return p1;
      }

      void
      setP2 (PointT p2)
      {
        p2Set = true;
        this->p2 = p2;
      }

      PointT
      getP2 ()
      {
        return p2;
      }

      bool
      isValid ()
      {
        if (p1Set == true && p2Set == true)
        {
          return true;
        }
        return false;
      }

      LineSegment& operator=(const LineSegment& l) {
        p1 = l.p1;
        p2 = l.p2;
        p1Set = l.p1Set;
        p2Set = l.p2Set;
        return (*this);
      }

      inline bool isP1Set() {
        return p1Set;
      }

      inline bool isP2Set() {
        return p2Set;
      }
    };

}

#define PCL_INSTANTIATE_LineSegment(T) template class PCL_EXPORTS pcl::LineSegment<T>;

#endif
