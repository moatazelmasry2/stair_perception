#ifndef FHG_PCL_POINTTYPES_H_
#define FHG_PCL_POINTTYPES_H_

#include <pcl/point_types.h>
#include "pcl/impl/instantiate.hpp"



namespace pcl
{

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
      int id;
      float confidence;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct EIGEN_ALIGN16 PointMoXYZRGB : public _PointMoXYZRGB
    {
      inline PointMoXYZRGB ()
      {
        _unused = 0;
        confidence = 1.0;
      }
      inline PointMoXYZRGB (uint8_t _r, uint8_t _g, uint8_t _b)
      {
        r = _r;
        g = _g;
        b = _b;
        _unused = 0;
        confidence = 1.0;
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
        int id;
        float confidence;
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
          confidence = 1.0;
        }
      };
//#include "pcl/common/impl/fhg_point_types.hpp"


  inline std::ostream&
  operator << (std::ostream& os, const PointMoXYZRGB& p)
  {
    os << "PointMoXYZRGB(" << p.x << "," << p.y << "," << p.z << " - " << p.rgb << ")";
    return (os);
  }


  inline std::ostream&
  operator << (std::ostream& os, const PointMoXYZRGBNormal& p)
  {
    os << "PointMoXYZRGBNormal(" << p.x << "," << p.y << "," << p.z << " - " << p.rgb << " - " << p.normal[0] << "," << p.normal[1] << ","
        << p.normal[2] << " - " << p.r << ", " << p.g << ", " << p.b << " - " << p.curvature << ")";
    return (os);
  }

}//end of pcl namespace


POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointMoXYZRGB,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (int, id, id)
    (float, confidence, confidence)
)

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointMoXYZRGB, pcl::_PointMoXYZRGB)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointMoXYZRGBNormal,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
    (int, id, id)
    (float, confidence, confidence)
)

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointMoXYZRGBNormal, pcl::_PointMoXYZRGBNormal)

#define PointMoTypes \
  (pcl::PointMoXYZRGB) \
  (pcl::PointMoXYZRGBNormal)

#endif
