/*
 * step.h
 *
 *  Created on: Jul 29, 2012
 *      Author: elmasry
 */

#ifndef STEP_H_
#define STEP_H_

#include "pcl/types/plane3d.h"
#include "pcl/types/riser.h"
#include "pcl/types/tread.h"

namespace pcl
{
  template<typename PointT>
  class Step
  {

    protected:
      Tread<PointT> tread;
      Riser<PointT> riser;
      bool riserExist, treadExist;
      size_t id;

      void calculateCenter ()
      {
        if (this->hasTread () && this->hasRiser ())
        {
          center = (tread.getCenter () + riser.getCenter ()) / 2;
        }
        else if (this->hasTread ())
        {
          center = tread.getCenter ();
        }
        else if (this->hasRiser ())
        {
          center = riser.getCenter ();
        }
      }
    public:

      Eigen::Vector3f center;
      Step ()
      {
        treadExist = false;
        riserExist = false;
        center.Zero ();
      }

      Step (size_t id)
      {
        this->id = id;
        treadExist = false;
        riserExist = false;
        center.Zero ();
      }

      Step (const Step<PointT>& step)
      {
        Step<PointT>::operator= (step);
      }

      Step&
      operator= (const Step<PointT>& step)
      {
        this->treadExist = step.treadExist;
        this->riserExist = step.riserExist;
        this->tread = step.tread;
        this->riser = step.riser;
        this->center = step.center;
        this->id = step.id;
        return (*this);
      }

      bool operator== (const Step<PointT>& step)
      {
        return id == step.id;
      }

      inline bool hasRiser () const
      {
        return riserExist;
      }

      inline bool hasTread () const
      {
        return treadExist;
      }

      /**
       * Riser can be set only once
       */
      inline bool setRiser (Riser<PointT> riser)
      {
        riserExist = true;
        this->riser = riser;
        if (!treadExist)
        {
          center = riser.center;
        }
        else
        {
          center = (riser.center + tread.center) / 2;
        }
        return true;
      }

      inline bool setTread (Tread<PointT> tread)
      {
        treadExist = true;
        this->tread = tread;
        if (!riserExist)
        {
          center = tread.center;
        }
        else
        {
          center = (tread.center + riser.center) / 2;
        }
        return true;
      }

      inline const Tread<PointT>&
      getTread () const
      {
        return tread;
      }

      inline const Riser<PointT>&
      getRiser () const
      {
        return riser;
      }

      bool operator< (const Step<PointT>& step) const
      {
        return (center[0] < step.center[0]);
      }

      Step<PointT>&
      operator+= (const Step<PointT>& step)
      {
        if (!this->hasTread () && step.hasTread ())
        {
          setTread (step.getTread ());
        }
        else if (this->hasTread () && step.hasTread ())
        {
          tread += step.getTread ();
        }

        if (!this->hasRiser () && step.hasRiser ())
        {
          setRiser (step.getRiser ());
        }
        else if (this->hasRiser () && step.hasRiser ())
        {
          riser += step.getRiser ();
        }
        return (*this);
      }

      bool isLanding () const
      {
        if (treadExist)
        {
          return tread.isLanding ();
        }
        return false;
      }

      Vector3f getCenter () const
      {
        return center;
      }

      void setCenter (Vector3f center)
      {
        this->center = center;
      }

      size_t getId () const
      {
        return id;
      }

      void setId (size_t id)
      {
        this->id = id;
      }

      void transform (Eigen::Matrix4f transformation)
      {
        if (hasRiser ())
        {
          riser.transform (transformation);
        }

        if (hasTread ())
        {
          tread.transform (transformation);
        }
        calculateCenter ();
      }

      void logStep () const
      {
//        printf ("Step: %d", (int) this->getId () );
//        printf (", center=(%f,%f,%f)", this->getCenter ()[0], this->getCenter ()[1], this->getCenter ()[2]);
        if (this->hasTread ())
        {
//          printf (", hasTread, l=%f, lDepth=%f, rDepth=%f, size=%d, ", this->getTread ().length, this->getTread ().lDepth,
//              this->getTread ().rDepth, this->getTread ().getCloud ()->size ());
          this->getTread ().logPlane ();
        }
        if (this->hasRiser ())
        {
//          printf (", hasRiser, l=%f, Height=%f, size=%d, ", this->getRiser ().length, this->getRiser ().height,
//              this->getRiser ().getCloud ()->size ());
          this->getRiser ().logPlane ();
        }
      }

      bool collides (const Step<PointT>& step)
      {
        if (hasTread () && step.hasTread ())
        {

        }

        if (hasTread () && step.hasRiser ())
        {

        }
        if (hasRiser () && step.hasTread ())
        {

        }
        if (hasRiser () && step.hasRiser ())
        {

        }
        return false;
      }

  };
}

#define PCL_INSTANTIATE_Step(T) template class PCL_EXPORTS pcl::Step<T>;
#endif /* STEP_H_ */
