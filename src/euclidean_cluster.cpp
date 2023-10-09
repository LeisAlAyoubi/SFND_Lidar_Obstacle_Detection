#include "euclidean_cluster.h"

template <typename PointT>
void Proximity(const typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, int pointIdx, std::vector<int>& cluster, float distanceTol, std::vector<bool>& processed)
{
    processed[pointIdx] = true;
    cluster.push_back(pointIdx);

    std::vector<int> nearbyPoints = tree->search({cloud->points[pointIdx].x, cloud->points[pointIdx].y, cloud->points[pointIdx].z}, distanceTol);

    for (int i : nearbyPoints)
    {
        if (!processed[i])
        {
            Proximity<PointT>(cloud, tree, i, cluster, distanceTol, processed);
        }
    }
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters_result;
    std::vector<bool> processed(cloud->size(), false);

    for (int i = 0; i < cloud->size(); ++i)
    {
        if (!processed[i])
        {
            std::vector<int> cluster_indices;
            Proximity<PointT>(cloud, tree, i, cluster_indices, distanceTol, processed);

            int clusterSize = cluster_indices.size();

            if (clusterSize >= minSize && clusterSize <= maxSize)
            {
                typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
                for (int index : cluster_indices)
                {
                    cluster_cloud->push_back(cloud->points[index]);
                }

                clusters_result.push_back(cluster_cloud);
            }
        }
    }

    return clusters_result;
}

