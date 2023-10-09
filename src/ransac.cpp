#include "ransac.h"
#include <cstdlib>
#include <ctime>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <cmath>

template <typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    for (int i = 0; i < maxIterations; i++)
    {
        std::unordered_set<int> inliners;

        // Randomly sample subset and fit plane
        while (inliners.size() < 3)
        {
            int randIndex = rand() % cloud->points.size();
            inliners.insert(randIndex);
        }

        auto itr = inliners.begin();
        PointT point1 = cloud->points[*itr];
        itr++;
        PointT point2 = cloud->points[*itr];
        itr++;
        PointT point3 = cloud->points[*itr];

        // Create plane spanning vectors
        Eigen::Vector3f v1(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z);
        Eigen::Vector3f v2(point3.x - point1.x, point3.y - point1.y, point3.z - point1.z);
        Eigen::Vector3f normal_vec = v1.cross(v2);

        // Plane coefficients
        float A = normal_vec[0];
        float B = normal_vec[1];
        float C = normal_vec[2];
        float D = -(A * point1.x + B * point1.y + C * point1.z);

        // Measure distance between every point and fitted plane
        for (int index = 0; index < cloud->points.size(); index++)
        {
            PointT point = cloud->points[index];
            float X1 = point.x;
            float Y1 = point.y;
            float Z1 = point.z;
            float distance = std::fabs(A * X1 + B * Y1 + C * Z1 + D) / std::sqrt(A * A + B * B + C * C);

            // If distance is smaller than threshold, count it as an inlier
            if (distance <= distanceTol)
            {
                inliners.insert(index);
            }
        }

        if (inliners.size() > inliersResult.size())
        {
            inliersResult = inliners;
        }
    }

    // Return indices of inliers from the fitted plane with the most inliers
    return inliersResult;
}
