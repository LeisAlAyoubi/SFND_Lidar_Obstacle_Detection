#ifndef EUCLIDEAN_CLUSTER_H
#define EUCLIDEAN_CLUSTER_H

#include "kdtree.h"

template <typename PointT>
void Proximity(const typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, int pointIdx, std::vector<int>& cluster, float distanceTol, std::vector<bool>& processed);

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize);
#endif // EUCLIDEAN_CLUSTER_H