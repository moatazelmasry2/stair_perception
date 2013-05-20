/*
 * region_segmentation.hpp
 *
 *  Created on: Jan 30, 2013
 *      Author: elmasry
 */

#ifndef REGION_SEGMENTATION_HPP_
#define REGION_SEGMENTATION_HPP_

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Dirk Holz, University of Bonn.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id:$
 *
 */

#include <pcl/segmentation/region_segmentation.h>

/** \brief Sort clusters method (for std::sort).
 * \ingroup segmentation
 */
inline bool pcl::segmentation::comparePointClusters(const pcl::segmentation::ClusterIndices& a, const pcl::segmentation::ClusterIndices& b)
{
    return (a.indices.size() < b.indices.size());
}

template<typename PointT> inline void pcl::segmentation::colorizeSegments(pcl::PointCloud<PointT>& cloud,
        const pcl::segmentation::VectorClusterIndices& clusters)
{
    const int nr_clusters = (int) clusters.size();
    for (int i = nr_clusters - 1; i >= 0; --i) // for (int i = 0; i < nr_clusters; ++i)
            {
        const float cluster_color = pcl::color::getRandomColor(0.5f, 2.8f);
        const int nr_points_cluster = (int) clusters[i].indices.size();
        for (int j = 0; j < nr_points_cluster; ++j)
            cloud.points[clusters[i].indices[j]].rgb = cluster_color;
    }
}

//template<typename PointT>

template<typename PointT>
inline void pcl::KdTree::CurvatureEstimation::setSearchMethod(const KdTreePtr &tree)
{
    tree_ = tree;
}


inline KdTreePtr pcl::KdTree::CurvatureEstimation::getSearchMethod()
{
    return (tree_);
}

inline void estimateCurvatures(PointCloudPtr cloud)
{
    if (!tree_) {
        //tree_.reset(new pcl::KdTreeFLANN<PointT>(false));
        tree_.reset(new KdTree(false));
        tree_->setInputCloud(cloud);
    }

    int nr_points = (int) cloud->size();
    for (int i = 0; i < nr_points; ++i) {
        const int& idx_point = i;
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        const int k = 20;
        int nr_neighbors = tree_->nearestKSearch(idx_point, k, nn_indices, nn_distances);
        if (nr_neighbors == 0) continue;

        for (int j = 0; j < nr_neighbors; ++j) {
            const int& idx_neighbor = nn_indices[j];

            const Eigen::Vector3f local_x_axis(1.0f, 0.0f, 0.0f);
            const Eigen::Vector3f normal = cloud->points[idx_point].getNormalVector3fMap();

            Eigen::Matrix3f R_local_frame = Eigen::Matrix3f::Zero();

//              R_local_frame.block<3,1>(0,2) = normal;
//              R_local_frame.block<3,1>(0,0) = (Eigen::Matrix3f::Identity() - normal*normal.transpose()) / ((Eigen::Matrix3f::Identity() - normal*normal.transpose()).norm());
//              R_local_frame.block<3,1>(0,1) = R_local_frame.block<3,1>(0,2).cross(R_local_frame.block<3,1>(0,0));

            Eigen::Vector3f local_coordinates = cloud->points[idx_neighbor].getVector3fMap() - cloud->points[idx_point].getVector3fMap();
//              local_coordinates *= R_local_frame.transpose();

        }
    }
}

protected:
using BasePCLBase::input_;
KdTreePtr tree_;
};

template<typename PointT>
class PlanePolygonalization: public PCLBase<PointT>
{
    typedef PCLBase<PointT> BasePCLBase;

public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    PlanePolygonalization()
    {
    }
    virtual ~PlanePolygonalization()
    {
    }

    void getPolygons(PointCloudPtr cloud, const pcl::segmentation::VectorClusterIndices& clusters, pcl::PolygonMesh& mesh)
    {
        PointCloudPtr output_cloud(new PointCloud);
        int nr_clusters = (int) (clusters.size());

        std::vector<Eigen::Vector4f> plane_models;
        plane_models.resize(nr_clusters);
        for (int i = 0; i < nr_clusters; ++i) {
            // compute plane parameters
            pcl::computePlaneParameters(*cloud, clusters[i].indices, plane_models[i]);
            projectPoints(*cloud, clusters[i].indices, plane_models[i]);

            // compute hull (convex, concave, alpha shape, ...)
            pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices(clusters[i]));
            PointCloudPtr cloud_hull(new PointCloud);
//            pcl::ConcaveHull<PointT> chull;
//            chull.setAlpha (0.1);
            pcl::ConvexHull<PointT> chull;
            chull.setInputCloud(cloud);
            chull.setIndices(plane_inliers);
            chull.reconstruct(*cloud_hull);

            int first_point = (int) output_cloud->points.size();
            int nr_points_hull = (int) cloud_hull->points.size();
            for (int point_idx = 0; point_idx < nr_points_hull; ++point_idx)
                output_cloud->points.push_back(cloud_hull->points[point_idx]);
            int last_point = (int) output_cloud->points.size();

            pcl::Vertices polygon;
            for (int point_idx = first_point; point_idx < last_point; ++point_idx)
                polygon.vertices.push_back(point_idx);
            for (int point_idx = first_point; point_idx < last_point; ++point_idx)
                output_cloud->points[point_idx].rgb = cloud->points[clusters[i].indices[0]].rgb;
            mesh.polygons.push_back(polygon);
        }
        pcl::toROSMsg(*output_cloud, mesh.cloud);
    }

protected:
    using BasePCLBase::input_;
};

/**
 * \brief Abstract implementation of a clustering algorithm based on growing regions from initial seeds.
 * \author Dirk Holz
 * \ingroup segmentation
 */
template<typename PointT>
class RegionSegmentation: public PCLBase<PointT>
{
    typedef PCLBase<PointT> BasePCLBase;

public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    //typedef typename pcl::KdTree<PointT> KdTree;
    typedef pcl::search::Search<PointT> KdTree;
    //typedef typename pcl::KdTree<PointT>::Ptr KdTreePtr;
    typedef typename pcl::search::Search<PointT>::Ptr KdTreePtr;

    typedef typename pcl::NoiseModel<PointT> NoiseModel;
    typedef typename pcl::NoiseModel<PointT>::Ptr NoiseModelPtr;

    typedef PointIndices::Ptr PointIndicesPtr;
    typedef PointIndices::ConstPtr PointIndicesConstPtr;

public:

    static const int UNASSIGNED = -1;
    static const int INVALID_CLUSTER = -2;

    /** \brief Constructor */
    RegionSegmentation() :
            tree_(), min_pts_per_cluster_(1), max_pts_per_cluster_(std::numeric_limits<int>::max()), distance_tolerance_(0.01f), angle_tolerance_(
                    pcl::deg2rad(25.0f)), allow_multi_assignments_(false), make_complete_segmentation_(false)
    {

    }

    /** \brief Empty destructor */
    virtual ~RegionSegmentation()
    {
    }

    /** \brief Provide a pointer to the search object.
     * \param tree a pointer to the spatial search object.
     */
    inline void setSearchMethod(const KdTreePtr &tree)
    {
        tree_ = tree;
    }

    /** \brief Get a pointer to the search method used. */
    inline KdTreePtr getSearchMethod()
    {
        return (tree_);
    }

    /** \brief Provide a pointer to the noise model. */
    inline void setNoiseModel(const NoiseModelPtr& noise_model)
    {
        noise_model_ = noise_model;
    }
    /** \brief Get a pointer to the noise model. */
    inline NoiseModelPtr getNoiseModel()
    {
        return noise_model_;
    }

    /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid.
     * \param min_cluster_size the minimum cluster size
     */
    inline void setMinClusterSize(int min_cluster_size)
    {
        min_pts_per_cluster_ = min_cluster_size;
    }

    /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
    inline int getMinClusterSize()
    {
        return (min_pts_per_cluster_);
    }

    /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid.
     * \param max_cluster_size the maximum cluster size
     */
    inline void setMaxClusterSize(int max_cluster_size)
    {
        max_pts_per_cluster_ = max_cluster_size;
    }

    /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
    inline int getMaxClusterSize()
    {
        return (max_pts_per_cluster_);
    }

    /** \brief Set the spatial cluster tolerance as a measure in the L2 Euclidean space
     * \param tolerance the spatial cluster tolerance as a measure in the L2 Euclidean space
     */
    inline void setClusterTolerance(float tolerance)
    {
        distance_tolerance_ = tolerance;
    }

    /** \brief Get the spatial cluster tolerance as a measure in the L2 Euclidean space. */
    inline float getClusterTolerance()
    {
        return (distance_tolerance_);
    }

    /** \brief Set the angle tolerance as a measure between local surface normals
     * \param tolerance the angle tolerance as a measure between local surface normals
     */
    inline void setAngleTolerance(float tolerance)
    {
        angle_tolerance_ = tolerance;
    }

    /** \brief Get the angle tolerance as a measure between local surface normals. */
    inline float getAngleTolerance()
    {
        return (angle_tolerance_);
    }

    /** \brief Allow multi assignments.
     * allow_multi_assignments When true scluster can contain points that have been assigned to other clusters before.
     */
    inline void allowMultiAssignments(bool allow_multi_assignments)
    {
        allow_multi_assignments_ = allow_multi_assignments;
    }

    /** \brief Make a complete segmentation (allow mini clusters of size 1 and grow regions into already existing clusters) */
    inline void makeCompleteSegmentations(bool make_complete_segmentation)
    {
        make_complete_segmentation_ = make_complete_segmentation;
    }

    /** \brief Cluster extraction in a PointCloud given by <setInputCloud (), setIndices ()>
     * \param clusters the resultant point clusters
     */
    inline void extract(VectorClusterIndices& clusters)
    {
        // Empty clusters
        clusters.clear();

        // Abort if either the cloud or the set of indices is empty (or simply not given)
        if (!initCompute() || (input_ != 0 && input_->points.empty()) || (indices_ != 0 && indices_->empty())) return;

        // Initialize the spatial locator
        if (!tree_) {
            //tree_.reset(new pcl::KdTreeFLANN<PointT>(false));
            tree_.reset(new ::pcl::search::KdTree<PointT>(false));
            tree_->setInputCloud(input_, indices_); // TODO: make proper indices
        }

        // Initialize the noise model
        if (!noise_model_) {
            noise_model_.reset(new pcl::NoiseModel<PointT>);
            noise_model_->setInputCloud(input_, indices_);
        }

        // Call the segmentation function of the actual algorithm implementation
        extractClusters(clusters);

        // sort clusters w.r.t. size (largest one first)
        std::sort(clusters.rbegin(), clusters.rend(), pcl::segmentation::comparePointClusters);

        deinitCompute();
    }

protected:
    using BasePCLBase::input_;
    using BasePCLBase::indices_;
    using BasePCLBase::initCompute;
    using BasePCLBase::deinitCompute;

    /** \brief A pointer to the spatial search object. */
    KdTreePtr tree_;

    /** \brief A pointer to the noise model. */
    NoiseModelPtr noise_model_;

    /** \brief The minimum number of points that a cluster needs to contain in order to be considered valid (default = 1). */
    int min_pts_per_cluster_;

    /** \brief The maximum number of points that a cluster needs to contain in order to be considered valid (default = MAXINT). */
    int max_pts_per_cluster_;

    /** \brief Distance tolerance */
    float distance_tolerance_;

    /** \brief Angle (between normals) tolerance */
    float angle_tolerance_;

    /** \brief Whether or not points can get assigned to multiple clusters .*/
    bool allow_multi_assignments_;

    /** \brief Make a complete segmentation (allow mini clusters of size 1 and grow regions into already existing clusters) */
    bool make_complete_segmentation_;

    /** \brief Class getName method. */
    virtual std::string getClassName() const
    {
        return ("BaseRegionSegmentation");
    }

    /** \brief EXTRACT clusters using the built-in region growing algorithm */
    virtual inline void extractClusters(VectorClusterIndices& clusters)
    {
        int nr_points = input_->width * input_->height;
        std::vector<int> assigned_to_cluster(nr_points, UNASSIGNED);
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;

        std::queue<int> seed_queue;
        std::queue<int> processing_queue;
        std::vector<bool> is_on_processing_queue(nr_points, false);
        std::vector<bool> is_in_current_cluster(nr_points, false);

        ClusterIndices current_cluster;
        current_cluster.header = input_->header;
        current_cluster.indices.reserve(nr_points);
        int counts = 0;

        // just process all points
        for (int i = 0; i < nr_points; ++i)
            seed_queue.push(i);

//          // ordered by curvature
//          std::vector< std::pair<int, float> > indices_list;
//          indices_list.resize(nr_points);
//          for (int i = 0; i < nr_points; ++i)
//            indices_list.push_back(std::pair<int,float>(i, input_->points[i].curvature));
//          std::sort(indices_list.begin(), indices_list.end(),
//                    boost::bind(&std::pair<int, float>::second, _1) >
//                    boost::bind(&std::pair<int, float>::second, _2));
////          const int nr_points_to_use_as_seeds = (int)(indices_list.size()*0.98f);
//          const int nr_points_to_use_as_seeds = (int)indices_list.size();
//          for (int i = 0; i < nr_points_to_use_as_seeds; ++i)
//            seed_queue.push(indices_list[i].first);

        while (!seed_queue.empty()) {
            // get seed point for current segment
            const int& seed_point = seed_queue.front();
            seed_queue.pop();

            // neglect seed points that are already assigned to clusters
            if (assigned_to_cluster[seed_point] != UNASSIGNED) continue;

            initCluster(seed_point);

            processing_queue.push(seed_point);
            is_on_processing_queue[seed_point] = true;

            while (!processing_queue.empty()) {
                // get current point and check if it is still unprocessed
                const int& current_point = processing_queue.front();
                processing_queue.pop();
                is_on_processing_queue[current_point] = false;

                // check if point is already contained in the current cluster
                if (is_in_current_cluster[current_point]) continue;

                // check if point is already contained in another cluster
                bool already_assigned = (assigned_to_cluster[current_point] != UNASSIGNED);
                if (already_assigned && !allow_multi_assignments_) continue;

                // check if point fits into the current cluster (and add it)
                if (!fitsCluster(current_point)) continue;

                // update model and add point to cluster
                updateCluster(current_point);
                current_cluster.indices.push_back(current_point);
                is_in_current_cluster[current_point] = true;

//              if (min_pts_per_cluster_ < (int)(current_cluster.indices.size()))
//              {
//                while (!processing_queue.empty())
//                  processing_queue.pop();
//                for (int i = 0; i < nr_points; ++i)
//                  is_on_processing_queue[i] = false;
//                break;
//              }

                // grow region (if and only if the point was not already assigned to another cluster beforehand
                if (already_assigned && !make_complete_segmentation_) continue;
                int nr_neighbors = getNeighbors(current_point, nn_indices, nn_distances);
                if (nr_neighbors > 0) {
                    for (int i = 0; i < nr_neighbors; ++i) {
                        const int& neighbor = nn_indices[i];
                        if ((current_point == neighbor) || is_in_current_cluster[neighbor] || is_on_processing_queue[neighbor]
                                || ((assigned_to_cluster[current_point] != UNASSIGNED) && !allow_multi_assignments_)
                                || !fitsNeighbor(nn_indices[i])) continue;

                        // put the neighbor on the processing cluster
                        processing_queue.push(neighbor);
                        is_on_processing_queue[neighbor] = true;
                    }
                }
            }

            // add the cluster to the found clusters
            int cluster_size = (int) (current_cluster.indices.size());
            if (make_complete_segmentation_ || ((cluster_size >= min_pts_per_cluster_) && (cluster_size <= max_pts_per_cluster_))) {
                int nr_current_cluster = (int) clusters.size();
                for (int i = 0; i < cluster_size; ++i)
                    assigned_to_cluster[current_cluster.indices[i]] = nr_current_cluster;
                clusters.push_back(current_cluster);
            } else
            // mark cluster as invalid (to avoid re-processing the contained points)
            for (int i = 0; i < cluster_size; ++i)
                assigned_to_cluster[current_cluster.indices[i]] = INVALID_CLUSTER;

            for (int i = 0; i < cluster_size; ++i)
                is_in_current_cluster[current_cluster.indices[i]] = false;
            current_cluster.indices.clear();
        }
    }

    virtual inline int getNeighbors(const int& point_idx, std::vector<int>& nn_indices, std::vector<float>& nn_distances)
    {
        nn_indices.clear();
        nn_distances.clear();
//          const float radius = 0.01f;
//          return tree_->radiusSearch (point_idx, radius, nn_indices, nn_distances);
        const int k = 100;
        return tree_->nearestKSearch(point_idx, k, nn_indices, nn_distances);
    }

    /** \brief Check whether or not the given point fits into the cluster */
    virtual inline bool fitsCluster(const int& point_idx)
    {
        return true;
    }
    ;

    /** \brief Initialize the model for the cluster */
    virtual inline void initCluster(const int& seed_point_idx)
    {
    }
    ;

    /** \brief Update the cluster model using the given point. */
    virtual inline void updateCluster(const int& point_idx)
    {
    }
    ;

    /** \brief Check whether or not the given neighbor fits into the cluster */
    virtual inline bool fitsNeighbor(const int& point_idx)
    {
        return true;
    }
    ;

};

template<typename PointT>
class InitialNormalRegionSegmentation: public RegionSegmentation<PointT>
{
    typedef RegionSegmentation<PointT> RegionSegmentationBase;
    using RegionSegmentationBase::input_; /* TODO: only give points instead of indices */
    using RegionSegmentationBase::angle_tolerance_;

public:
    InitialNormalRegionSegmentation()
    {
    }

protected:

    std::string getClassName() const
    {
        return ("InitialNormalRegionSegmentation");
    }

    inline bool fitsCluster(const int& point_idx)
    {
        return (fabs(normal_.dot(input_->points[point_idx].getNormalVector3fMap())) > cos_angle_tolerance_);
//          return true;
    }

    inline void initCluster(const int& seed_point_idx)
    {
        cos_angle_tolerance_ = cos(angle_tolerance_);
        normal_ = input_->points[seed_point_idx].getNormalVector3fMap();
    }

    float cos_angle_tolerance_;
    Eigen::Vector3f normal_;
};

template<typename PointT>
class LastNormalRegionSegmentation: public RegionSegmentation<PointT>
{
    typedef RegionSegmentation<PointT> RegionSegmentationBase;
    using RegionSegmentationBase::input_; /* TODO: only give points instead of indices */
    using RegionSegmentationBase::angle_tolerance_;

public:
    LastNormalRegionSegmentation()
    {
    }

protected:

    std::string getClassName() const
    {
        return ("LastNormalRegionSegmentation");
    }

    inline void initCluster(const int& seed_point_idx)
    {
        cos_angle_tolerance_ = cos(angle_tolerance_);
        last_normal_ = input_->points[seed_point_idx].getNormalVector3fMap();
    }

    inline void updateCluster(const int& point_idx)
    {
        last_normal_ = input_->points[point_idx].getNormalVector3fMap();
    }

    inline bool fitsNeighbor(const int& point_idx)
    {
        return (fabs(last_normal_.dot(input_->points[point_idx].getNormalVector3fMap())) > cos_angle_tolerance_);
    }

    float cos_angle_tolerance_;
    Eigen::Vector3f last_normal_;
};

template<typename PointT>
class AverageNormalRegionSegmentation: public RegionSegmentation<PointT>
{
    typedef RegionSegmentation<PointT> RegionSegmentationBase;
    using RegionSegmentationBase::input_;
    using RegionSegmentationBase::indices_;
    using RegionSegmentationBase::angle_tolerance_;

public:
    AverageNormalRegionSegmentation()
    {
    }

protected:

    std::string getClassName() const
    {
        return ("AverageNormalRegionSegmentation");
    }

    inline bool fitsCluster(const int& point_idx)
    {
        Eigen::Vector3f normal_cluster = normal_sum_.normalized();
        return (fabs(normal_cluster.dot(input_->points[point_idx].getNormalVector3fMap())) > cos_angle_tolerance_);
    }

    inline void initCluster(const int& seed_point_idx)
    {
        cos_angle_tolerance_ = cos(angle_tolerance_);
        normal_sum_ = input_->points[seed_point_idx].getNormalVector3fMap();
    }

    inline void updateCluster(const int& point_idx)
    {
        normal_sum_ += input_->points[point_idx].getNormalVector3fMap();
    }

    float cos_angle_tolerance_;
    Eigen::Vector3f normal_sum_;
};

template<typename PointT>
class ApproximatePlaneRegionSegmentation: public RegionSegmentation<PointT>
{
    typedef RegionSegmentation<PointT> RegionSegmentationBase;
    using RegionSegmentationBase::input_;
    using RegionSegmentationBase::angle_tolerance_;
    using RegionSegmentationBase::noise_model_;

public:
    ApproximatePlaneRegionSegmentation() :
            sum_normal_(Eigen::Vector3f::Zero()), sum_xyz_(Eigen::Vector3f::Zero()), sum_square_error_(0.0f), nr_points_(0), max_distance_(
                    0.01f), cos_angle_tolerance_(cos(angle_tolerance_))
    {
    }

protected:

    std::string getClassName() const
    {
        return ("ApproximatePlaneRegionSegmentation");
    }

    inline bool fitsCluster(const int& point_idx)
    {
        Eigen::Vector3f normal = sum_normal_.normalized();
        Eigen::Vector3f centroid = sum_xyz_ / (float) nr_points_;
        pcl::PlaneModel plane(normal, centroid);

        const PointT& point = input_->points[point_idx];
        const float sigma = noise_model_->getStdDev(point_idx);
        const float weight = noise_model_->getWeight(point_idx);
        const float distance = point.getVector3fMap().norm();

        const float mean_square_error = sum_square_error_ / (float) nr_points_;

        max_distance_ = 2.0f * sigma;
//          max_distance_ = sigma;
        const float dist_to_plane = fabs(pcl::distPointPlane(point, plane));
        bool check_distance = (dist_to_plane < max_distance_);

//          float adaptive_angle_tolerance = 5.0f + distance*5.0f;
//          cos_angle_tolerance_ = cos(pcl::deg2rad(adaptive_angle_tolerance));
//          bool check_normal = (plane.normal.dot(point.getNormalVector3fMap()) > cos_angle_tolerance_);
        bool check_normal = (fabs(plane.normal.dot(point.getNormalVector3fMap())) > cos_angle_tolerance_);

        if (check_distance && check_normal) {
            sum_square_error_ += dist_to_plane * dist_to_plane;
            return true;
        }
        return false;
    }

    inline void initCluster(const int& point_idx)
    {
        const PointT& point = input_->points[point_idx];
        sum_normal_ = point.getNormalVector3fMap();
        sum_xyz_ = point.getVector3fMap();
        sum_square_error_ = 0;
        nr_points_ = 1;
        cos_angle_tolerance_ = cos(angle_tolerance_);
    }

    inline void updateCluster(const int& point_idx)
    {
        const PointT& point = input_->points[point_idx];
        sum_normal_ += point.getNormalVector3fMap();
        sum_xyz_ += point.getVector3fMap();
        ++nr_points_;
    }

    Eigen::Vector3f sum_normal_;
    Eigen::Vector3f sum_xyz_;
    float sum_square_error_;
    int nr_points_;

    float max_distance_;
    float cos_angle_tolerance_;
};

template<typename PointT>
class ProbabilisticPlaneRegionSegmentation: public RegionSegmentation<PointT>
{
    typedef RegionSegmentation<PointT> RegionSegmentationBase;
    using RegionSegmentationBase::input_;
    using RegionSegmentationBase::indices_;
    using RegionSegmentationBase::min_pts_per_cluster_;
    using RegionSegmentationBase::angle_tolerance_;
    using RegionSegmentationBase::noise_model_;

public:
    ProbabilisticPlaneRegionSegmentation() :
            plane_(new pcl::PlaneModelIncremental), max_distance_(0.01f), max_mse_(1.5 * max_distance_ * max_distance_), cos_angle_tolerance_(
                    cos(angle_tolerance_))
    {
    }

    inline void setMaxMSE(float max_mse)
    {
        max_mse_ = max_mse;
    }
    ;
    inline float getMaxMSE() const
    {
        return max_mse_;
    }
    ;

protected:

    std::string getClassName() const
    {
        return ("ProbabilisticPlaneRegionSegmentation");
    }

    inline void initCluster(const int& seed_point_idx)
    {
        plane_->reset();
        cos_angle_tolerance_ = cos(angle_tolerance_);
    }

    inline bool fitsCluster(const int& point_idx)
    {
        const PointT& point = input_->points[point_idx];
        const float sigma = noise_model_->getStdDev(point_idx);
        const float weight = noise_model_->getWeight(point_idx);

        plane_->addPoint(point, weight);

        if (plane_->getNumberOfPoints() < 7) return true; // we need a minimum number of points, but simply wait until enough neighbors have been added!

        pcl::PlaneModel plane;
        float lambda = plane_->getOptimalPlane(*plane_, plane);
        float mse = plane_->getMSE(*plane_, plane);

        float dist_to_plane = fabs(pcl::distPointPlane(point, plane));

//          if  (plane_->getNumberOfPoints() == min_pts_per_cluster_)
//          {
//            std::cout << "PLANE : " << lambda << ' ' << mse << ' ' << dist_to_plane << ' ';
//            std::cout << " " << plane.normal(0) << ' ' << plane.normal(1) << ' ' << plane.normal(2) << "   " << plane.distance << std::endl;
//          }

        max_distance_ = sigma;
//          max_distance_ = 0.75f * sigma;
        max_mse_ = max_distance_ * max_distance_;

        bool check_mse = (mse < max_mse_);
        bool check_distance = (dist_to_plane < max_distance_);
        bool check_normal = (plane.normal.dot(point.getNormalVector3fMap()) > cos_angle_tolerance_);

        if (check_mse && check_distance && check_normal) return true;

        plane_->removePoint(point, weight);
        return false;
    }

    boost::shared_ptr<pcl::PlaneModelIncremental> plane_;

    float max_distance_;
    float max_mse_;
    float cos_angle_tolerance_;
};

template<typename PointT>
class PCLEuclideanRegionSegmentation: public RegionSegmentation<PointT>
{
    typedef RegionSegmentation<PointT> RegionSegmentationBase;
    using RegionSegmentationBase::input_;
    using RegionSegmentationBase::indices_;
    using RegionSegmentationBase::tree_;
    using RegionSegmentationBase::min_pts_per_cluster_;
    using RegionSegmentationBase::max_pts_per_cluster_;
    using RegionSegmentationBase::distance_tolerance_;

public:
    PCLEuclideanRegionSegmentation()
    {
    }

protected:
    std::string getClassName() const
    {
        return ("PCLEuclideanRegionSegmentation");
    }

    void extractClusters(VectorClusterIndices& clusters)
    {
        extractEuclideanClusters(*input_, *indices_, tree_, distance_tolerance_, clusters, min_pts_per_cluster_, max_pts_per_cluster_);
    }
};

template<typename PointT>
class PCLNormalRegionSegmentation: public RegionSegmentation<PointT>
{
    typedef RegionSegmentation<PointT> RegionSegmentationBase;
    using RegionSegmentationBase::input_;
    using RegionSegmentationBase::indices_;
    using RegionSegmentationBase::tree_;
    using RegionSegmentationBase::min_pts_per_cluster_;
    using RegionSegmentationBase::max_pts_per_cluster_;
    using RegionSegmentationBase::distance_tolerance_;
    using RegionSegmentationBase::angle_tolerance_;

public:

    PCLNormalRegionSegmentation()
    {
    }

protected:

    std::string getClassName() const
    {
        return ("PCLNormalRegionSegmentation");
    }

    void extractClusters(VectorClusterIndices& clusters)
    {
        pcl::extractEuclideanClusters(*input_, *input_, distance_tolerance_, tree_, clusters, angle_tolerance_, min_pts_per_cluster_,
                max_pts_per_cluster_);
    }
};

}
}

#endif /* REGION_SEGMENTATION_HPP_ */
