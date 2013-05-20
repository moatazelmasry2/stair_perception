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
 * $Id$
 *
 */

#ifndef PCL_KDTREE_FIXED_NEIGHBORS_H_
#define PCL_KDTREE_FIXED_NEIGHBORS_H_

//#include "pcl/kdtree/kdtree.h"
#include <stdexcept>
#include <execinfo.h>
#include <stdlib.h>

#include <pcl/search/search.h>
#include <pcl/console/print.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl/common/mesh_utilities.h>

namespace pcl {

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b FixedNeighbors is a dummy class that uses previously specified neighbors (for compability reasons).
 * It provides access to fixed vectors of neighbors via a common interface.
 * \author Dirk Holz
 * \ingroup kdtree
 */
template<typename PointT>
class FixedNeighbors: public ::pcl::search::Search<PointT> {
	using ::pcl::search::Search<PointT>::input_;
	using ::pcl::search::Search<PointT>::indices_;
//    using KdTree<PointT>::min_pts_;

	typedef ::pcl::search::Search<PointT> BaseClass;
	typedef typename ::pcl::search::Search<PointT>::PointCloud PointCloud;
	typedef typename ::pcl::search::Search<PointT>::PointCloudConstPtr PointCloudConstPtr;

	typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
	typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

public:
	// Boost shared pointers
	typedef boost::shared_ptr<FixedNeighbors<PointT> > Ptr;
	typedef boost::shared_ptr<const FixedNeighbors<PointT> > ConstPtr;

	/** \brief Empty constructor for FixedNeighbors. Sets some internal values to their defaults. */
	FixedNeighbors() :
			::pcl::search::Search<PointT>(), max_distance_(0) {
	}

	inline Ptr makeShared() {
		return Ptr(new FixedNeighbors<PointT>(*this));
	}

	void
	setInputCloud(const PointCloudConstPtr &cloud,
			const IndicesConstPtr &indices = IndicesConstPtr());

	/** \brief Method not implemented
	 * \param p_q
	 * \param k
	 * \param k_indices
	 * \param k_distances
	 */
	int nearestKSearch(const PointT &p_q, int k, std::vector<int> &k_indices,
			std::vector<float> &k_distances) const {
		PCL_ERROR("%s: Method not implemented!\n", __PRETTY_FUNCTION__);
		return (false);
	}
	/** \brief Method not implemented
	 * \param p_q
	 * \param radius
	 * \param k_indices
	 * \param k_distances
	 * \param max_nn
	 */
	virtual int radiusSearch(const PointT &p_q, double radius,
			std::vector<int> &k_indices, std::vector<float> &k_distances,
			unsigned int max_nn = INT_MAX) const {
		PCL_ERROR("%s: Method not implemented!\n", __PRETTY_FUNCTION__);
		return (false);
	}

	/** \brief Approximate search for neighbors around the given query point within radius.
	 * \param cloud the point cloud data.
	 * \param index the index in \a cloud representing the query point.
	 * \param radius the maximum distance to search for neighbors in.
	 * \param k_indices the resultant point indices
	 * \param k_distances the resultant !squared! point distances
	 * \param max_nn maximum number of points to return
	 */
	virtual int radiusSearch(const PointCloud &cloud, int index, double radius,
			std::vector<int> &k_indices, std::vector<float> &k_distances,
			int max_nn = INT_MAX) const;

	/** \brief Approximate search for neighbors around the point from the given index within radius.
	 * \param index the index in \a cloud representing the query point.
	 * \param radius the maximum distance to search for neighbors in.
	 * \param k_indices the resultant point indices
	 * \param k_distances the resultant !squared! point distances
	 * \param max_nn maximum number of points to return
	 */
	virtual inline int radiusSearch(int index, double radius,
			std::vector<int> &k_indices, std::vector<float> &k_distances,
			int max_nn = INT_MAX) const {
		if (!input_) {
			PCL_ERROR(
					"[%s] Input dataset does not exist or wrong input dataset!\n", __PRETTY_FUNCTION__);
			return (false);
		}
		return (radiusSearch(*input_, index, radius, k_indices, k_distances,
				max_nn));
	}

	/** \brief Search for k-nearest neighbors for the given query point.
	 * \param cloud the point cloud data
	 * \param index the index in \a cloud representing the query point
	 * \param k the number of neighbors to search for (not used)
	 * \param k_indices the resultant point indices (must be resized to \a k beforehand!)
	 * \param k_distances \note this function does not return distances
	 */
	virtual int nearestKSearch(const PointCloud &cloud, int index, int k,
			std::vector<int> &k_indices, std::vector<float> &k_distances) const;

	/** \brief Search for k-nearest neighbors for the given query point.
	 * \param index the index representing the query point
	 * \param k the number of neighbors to search for (not used)
	 * \param k_indices the resultant point indices (must be resized to \a k beforehand!)
	 * \param k_distances \note this function does not return distances
	 */
	virtual int nearestKSearch(int index, int k, std::vector<int> &k_indices,
			std::vector<float> &k_distances) const {
		if (!input_) {
			PCL_ERROR(
					"[pcl::%s::nearestKSearch] Input dataset does not exist or wrong input dataset!\n", getName ().c_str ());
			return 0;
		}
		return (nearestKSearch(*input_, index, k, k_indices, k_distances));
	}

	/** \brief Provide a pointer to the input neighbor vectors */
	virtual inline void setNeighborhood(
			const NeighborhoodVectorConstPtr &neighbors) {
		neighbors_ = neighbors;
	}

	/** \brief Get a pointer to the input point cloud dataset. */
	inline NeighborhoodVectorConstPtr const getNeighborhood() {
		return (neighbors_);
	}

	/** \brief Set the maximum allowed distance between the query point and its k-nearest neighbors. */
	void setMaxDistance(float max_dist) {
		max_distance_ = max_dist;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Get the maximum allowed distance between the query point and its k-nearest neighbors. */
	float getMaxDistance() const {
		return (max_distance_);
	}

private:
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Class getName method. */
//	virtual std::string getName() const {
//		return ("FixedNeighbors");
//	}

	virtual const std::string&
	getName() const {
		return ("FixedNeighbors");
	}

	/** \brief Maximum allowed distance between the query point and its k-neighbors. */
	float max_distance_;

	NeighborhoodVectorConstPtr neighbors_;

	std::vector<bool> used_;

};
}

#endif  //#ifndef _PCL_KDTREE_FIXED_NEIGHBORS_H_
