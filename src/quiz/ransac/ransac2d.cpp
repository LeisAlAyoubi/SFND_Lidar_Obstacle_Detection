/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations
	for (int i = 0; i < maxIterations; i++)
	{

		std::unordered_set<int> inliners;

		// Randomly sample subset and fit line
		while (inliners.size() < 3)
		{
			int randIndex = rand() % cloud->points.size();
			inliners.insert(randIndex);
		}

		auto itr = inliners.begin();
		pcl::PointXYZ point1 = cloud->points[*itr];
		itr++;
		pcl::PointXYZ point2 = cloud->points[*itr];
		itr++;
		pcl::PointXYZ point3 = cloud->points[*itr];

		// Create plane spanning vectors
		Eigen::Vector3f v1(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z);
		Eigen::Vector3f v2(point3.x - point1.x, point3.y - point1.y, point3.z - point1.z);
		// normal vector to the plane
		Eigen::Vector3f normal_vec = v1.cross(v2);

		// Line coefficients
		float A = normal_vec[0];
		float B = normal_vec[1];
		float C = normal_vec[2];
		float D = -(A * point1.x + B * point1.y + C * point1.z);

		// Measure distance between every point and fitted line
		for (int index = 0; index < cloud->points.size(); index++)
		{

			pcl::PointXYZ point = cloud->points[index];
			float X1 = point.x;
			float Y1 = point.y;
			float Z1 = point.z;
			float distance = std::fabs(A * X1 + B * Y1 + C * Z1 + D) / std::sqrt(A * A + B * B + C * C);

			// If distance is smaller than threshold count it as inlier
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

	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
