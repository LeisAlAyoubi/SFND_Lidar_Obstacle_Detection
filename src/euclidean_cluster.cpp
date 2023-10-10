#include "euclidean_cluster.h"

/**
 * Recursive function to find nearby points within a specified Euclidean distance.
 *
 * @param points The list of points to search.
 * @param tree A pointer to the KD-Tree data structure.
 * @param pointIdx The index of the current point to start the search from.
 * @param cluster A vector to store the indices of points in the cluster.
 * @param distanceTol The maximum Euclidean distance to consider for proximity.
 * @param processed A vector to keep track of processed points.
 */
void Proximity(const std::vector<std::vector<float>>& points, KdTree* tree, int pointIdx, std::vector<int>& cluster, float distanceTol, std::vector<bool>& processed)
{
    processed[pointIdx] = true;
    cluster.push_back(pointIdx);

    std::vector<int> nearbyPoints = tree->search(points[pointIdx], distanceTol);

    for (int i : nearbyPoints)
    {
        if (!processed[i])
        {
            Proximity(points, tree, i, cluster, distanceTol, processed);
        }
    }
}

/**
 * Cluster points in the input list based on Euclidean proximity using a KD-Tree.
 *
 * @param points The list of points to cluster.
 * @param tree A pointer to the KD-Tree data structure.
 * @param distanceTol The maximum Euclidean distance to consider for clustering.
 * @return A vector of vectors, where each inner vector represents a cluster of points.
 */
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    for (int i = 0; i < points.size(); ++i)
    {
        if (!processed[i])
        {
            std::vector<int> cluster;
            Proximity(points, tree, i, cluster, distanceTol, processed);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

