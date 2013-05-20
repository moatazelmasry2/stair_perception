#ifndef PCL_MYICP_H_
#define PCL_MYICP_H_

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

namespace pcl
{

  template<typename PointSource, typename PointTarget>
  class PCL_EXPORTS MyICP : public IterativeClosestPointWithNormals<PointSource, PointTarget>
  {
      using Registration<PointSource, PointTarget>::correspondences_;

    public:

      CorrespondencesPtr getCorrespondances ()
      {
        return correspondences_;
      }
  };

  template<typename PointSource, typename PointTarget, typename Scalar = float>
  class MyICPNonLinear : public IterativeClosestPointNonLinear<PointSource, PointTarget>
  {
      using Registration<PointSource, PointTarget>::correspondences_;

    public:

      CorrespondencesPtr getCorrespondances ()
      {
        return this->correspondences_;
      }
  };
}

#define PCL_INSTANTIATE_MyICP(Source,Target) template class PCL_EXPORTS pcl::MyICP<Source,Target>;
#define PCL_INSTANTIATE_MyICPNonLinear(Source,Target) template class PCL_EXPORTS pcl::MyICPNonLinear<Source,Target>;

#endif
