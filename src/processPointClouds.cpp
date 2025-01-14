// PCL lib Functions for processing point clouds

#include <unordered_set>
#include "processPointClouds.h"
#include "ransac.h"
#include "ransac.cpp"
#include "euclidean_cluster.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    // crop region of interest
    typename pcl::PointCloud<PointT>::Ptr cloud_roi(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> box;
    box.setInputCloud(cloud_filtered);
    box.setMin(minPoint);
    box.setMax(maxPoint);
    box.filter(*cloud_roi);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_roi;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obst_cloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    // Create the filtering object
    extract.setNegative(true);
    extract.filter(*obst_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obst_cloud, plane_cloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // // Create the segmentation object
    // pcl::SACSegmentation<PointT> seg;
    // // Optional
    // seg.setOptimizeCoefficients(true);
    // // Mandatory
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);
    // // Segment the largest planar component from the cloud
    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);
    // if (inliers->indices.size() == 0)
    // {
    //     std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    // }
    
    std::unordered_set<int> inlier_indices = Ransac3D<PointT>(cloud, maxIterations, distanceThreshold);
    // Iterate through the unordered_set and add elements to PointIndices
    for (const int &index : inlier_indices)
    {
        inliers->indices.push_back(index);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, bool useCustom)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters_result;
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    if (!useCustom)
    {
        // PCL PROVIDED CLUSTERING
        // Creating the KdTree object for the search method of the extraction
        std::vector<pcl::PointIndices> cluster_indices;
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(clusterTolerance);
        ec.setMinClusterSize(minSize);
        ec.setMaxClusterSize(maxSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
        for (const auto &cluster : cluster_indices)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            for (const auto &index : cluster.indices)
            {
                cloud_cluster->push_back((*cloud)[index]);
            }
            clusters_result.push_back(cloud_cluster);
        }
    }
    else
    {
        // CUSTOM CLUSTERING
        // Create a KD-Tree for efficient nearest neighbor searches
        KdTree *tree = new KdTree;
        std::vector<std::vector<float>> points;

        // Insert each point into the KD-Tree with its corresponding index and into the points-vector
        for (int i = 0; i < cloud->size(); i++)
        {
            points.push_back({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
            tree->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}, i);
        }

        // Perform Euclidean clustering on the points using the KD-Tree
        std::vector<std::vector<int>> cluster_indices = euclideanCluster(points, tree, clusterTolerance);

        for (const std::vector<int> &cluster : cluster_indices)
        {
            int clusterSize = cluster.size();
            if (clusterSize >= minSize && clusterSize <= maxSize)
            {
                typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
                for (int index : cluster)
                {
                    cluster_cloud->push_back(cloud->points[index]);
                }
                clusters_result.push_back(cluster_cloud);
            }
        }
        // clusters_result = euclideanCluster<PointT>(cloud, tree, clusterTolerance, minSize, maxSize);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters_result.size() << " clusters" << std::endl;

    return clusters_result;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}