#ifndef PCL_REGISTRATION_DIRK_H_
#define PCL_REGISTRATION_DIRK_H_

#include <pcl/point_cloud.h>
#if (!defined PCL_MINOR_VERSION || (PCL_MINOR_VERSION < 3))
#include <pcl/common/transform.h>
#include <pcl/registration/correspondence_types.h>
namespace pcl
{
  typedef pcl::registration::Correspondences Correspondences;
  typedef pcl::registration::CorrespondencesPtr CorrespondencesPtr;
}
#else
#include <pcl/common/eigen.h>
#include <pcl/correspondence.h>
#endif
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/ia_ransac.h>

#include <pcl/common/time.h>

namespace pcl
{
    namespace registration
    {

      template <typename SourceT, typename TargetT> inline void
      ICP(
          const typename pcl::PointCloud<SourceT>::Ptr& source_points,
          const typename pcl::PointCloud<TargetT>::Ptr& target_points,
          typename pcl::PointCloud<SourceT>::Ptr& source_aligned,
          Eigen::Matrix4f& transformation,
          pcl::CorrespondencesPtr& correspondences,
          std::vector<int>& rejected_indices,
          const std::vector<float>& distance_thresholds)
      {
        pcl::registration::CorrespondenceEstimation<SourceT, TargetT> correspondence_estimation;

        {
          //pcl::ScopeTime t("correspondence_estimation.setInputTarget(target_points)");
          correspondence_estimation.setInputTarget(target_points);
        }
        pcl::registration::TransformationEstimationSVD<SourceT, TargetT> trans_est;
        pcl::CorrespondencesPtr correspondences_not_rejected(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectorDistance corr_rejection_distance;

        pcl::transformPointCloud(*source_points, *source_aligned, transformation);

        const int nr_iterations = (int)distance_thresholds.size();
        for (int iteration = 0; iteration < nr_iterations; ++iteration)
        {
          std::cout << "ICP iteration " << iteration << std::endl;
          
          {
            //pcl::ScopeTime t("correspondence_estimation.determineCorrespondences(target_points)");
#if (!defined PCL_MINOR_VERSION || (PCL_MINOR_VERSION < 6))
            correspondence_estimation.setInputCloud(source_aligned);
#else
            correspondence_estimation.setInputSource(source_aligned);
#endif
            correspondence_estimation.determineCorrespondences(*correspondences, distance_thresholds[iteration] );
#if (!defined PCL_MINOR_VERSION || (PCL_MINOR_VERSION < 6))
            // in some older versions of CorrespondenceEstimation invalid correspondences
            // are returned for points not having a corresponding point in the target cloud
            pcl::CorrespondencesPtr correspondences_temp(new pcl::Correspondences);
            correspondences_temp->resize(correspondences->size());
            unsigned int nr_valid_correpsondences = 0;
            for (pcl::Correspondences::const_iterator iter = correspondences->begin(); iter != correspondences->end(); ++iter)
              if (iter->index_match != -1)
                (*correspondences_temp)[nr_valid_correpsondences++] = *iter;
            correspondences_temp->resize(nr_valid_correpsondences);
            correspondences.swap(correspondences_temp);
#endif
          }
          std::cout << correspondences->size() << " correspondences." << std::endl;
          
//           corr_rejection_distance.setInputCorrespondences(correspondences);
//           corr_rejection_distance.setMaximumDistance(distance_thresholds[iteration]);
// #if (!defined PCL_MINOR_VERSION || (PCL_MINOR_VERSION < 3))
//           corr_rejection_distance.getCorrespondeces(*correspondences_not_rejected);
// #else
//           corr_rejection_distance.getCorrespondences(*correspondences_not_rejected);
// #endif
//           correspondences.swap(correspondences_not_rejected);

          Eigen::Matrix4f transformation_estimated;
          trans_est.estimateRigidTransformation(*source_aligned, *target_points,
                                                *correspondences,
                                                transformation_estimated);

          pcl::transformPointCloud(*source_aligned, *source_aligned, transformation_estimated);
          transformation = transformation_estimated * transformation;
        }

        pcl::CorrespondencesPtr correspondences_temp (new pcl::Correspondences);
        pcl::CorrespondencesPtr correspondences_temp_temp (new pcl::Correspondences);
        correspondence_estimation.setInputCloud(source_aligned);
        correspondence_estimation.determineCorrespondences(*correspondences_temp, 100.f );
        corr_rejection_distance.setInputCorrespondences(correspondences_temp);
        corr_rejection_distance.setMaximumDistance(0.01f*0.01f);
#if (!defined PCL_MINOR_VERSION || (PCL_MINOR_VERSION < 3))
        corr_rejection_distance.getCorrespondeces(*correspondences_temp_temp);
#else
        corr_rejection_distance.getCorrespondences(*correspondences_temp_temp);
#endif        
        corr_rejection_distance.getRejectedQueryIndices(*correspondences_temp_temp, rejected_indices);

        //      std::cout << correspondences->size() << " correspondences." << std::endl;
        //    std::cout << transformation << std::endl;
      }




    }
}

#endif
