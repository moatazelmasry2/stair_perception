/*
 * fhg_point_types.hpp
 *
 *  Created on: Jan 28, 2013
 *      Author: elmasry
 */

#ifndef FHG_POINT_TYPES_HPP_
#define FHG_POINT_TYPES_HPP_


struct EIGEN_ALIGN16 _PointMoXYZRGB
  {
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    union
    {
      union
      {
        struct
        {
          uint8_t b;
          uint8_t g;
          uint8_t r;
          uint8_t _unused;
        };
        float rgb;
      };
      uint32_t rgba;
    };
    //int planeId;
    //int segmentId;
    //int edge;
    int id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct EIGEN_ALIGN16 PointMoXYZRGB : public _PointMoXYZRGB
  {
    inline PointMoXYZRGB ()
    {
      _unused = 0;
    }
    inline PointMoXYZRGB (uint8_t _r, uint8_t _g, uint8_t _b)
    {
      r = _r;
      g = _g;
      b = _b;
      _unused = 0;
    }

    inline Eigen::Vector3i getRGBVector3i () { return (Eigen::Vector3i (r, g, b)); }
    inline const Eigen::Vector3i getRGBVector3i () const { return (Eigen::Vector3i (r, g, b)); }
    inline Eigen::Vector4i getRGBVector4i () { return (Eigen::Vector4i (r, g, b, 0)); }
    inline const Eigen::Vector4i getRGBVector4i () const { return (Eigen::Vector4i (r, g, b, 0)); }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct EIGEN_ALIGN16 _PointMoXYZRGBNormal
    {
      PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
      PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
      union
      {
        struct
        {
          // RGB union
          union
          {
            struct
            {
              uint8_t b;
              uint8_t g;
              uint8_t r;
              uint8_t _unused;
            };
            float rgb;
            uint32_t rgba;
          };
          float curvature;
        };
        float data_c[4];
      };
      //int planeId;
      //int segmentId;
      //int edge;
      int id;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct PointMoXYZRGBNormal : public _PointMoXYZRGBNormal
    {
      inline
      PointMoXYZRGBNormal ()
      {
        _unused = 0;
        data[3] = 1.0f;
        data_n[3] = 0.0f;
      }
    };

#endif /* FHG_POINT_TYPES_HPP_ */
