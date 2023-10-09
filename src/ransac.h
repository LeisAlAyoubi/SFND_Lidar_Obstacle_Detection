#ifndef RANSAC_H
#define RANSAC_H

#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template <typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

#endif // RANSAC_H
