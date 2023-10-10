#ifndef EUCLIDEAN_CLUSTER_H
#define EUCLIDEAN_CLUSTER_H

#include "kdtree.h"

void Proximity(const std::vector<std::vector<float>>& points, KdTree* tree, int pointIdx, std::vector<int>& cluster, float distanceTol, std::vector<bool>& processed);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);
#endif // EUCLIDEAN_CLUSTER_H