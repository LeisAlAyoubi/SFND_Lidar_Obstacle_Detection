/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_point_cloud = lidar->scan();
    //renderRays(viewer, lidar->position, lidar_point_cloud);
    renderPointCloud(viewer, lidar_point_cloud, "Lidar point cloud");
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> point_cloud_processor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = point_cloud_processor.SegmentPlane(lidar_point_cloud, 500, 0.2);
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_cloud_processor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        point_cloud_processor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

        Box box = point_cloud_processor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1, Eigen::Vector4f(-15, -7, -5, 1), Eigen::Vector4f(25, 8, 5, 1));
    std::cout << "Number of points in the filtered cloud: " << filterCloud->points.size() << std::endl;
    renderPointCloud(viewer, filterCloud, "filterCloud");

    ProcessPointClouds<pcl::PointXYZI> point_cloud_processor;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = point_cloud_processor.SegmentPlane(filterCloud, 10, 0.2);

    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));



    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = point_cloud_processor.Clustering(segmentCloud.first, 0.5, 100, 1000);

    int clusterId = 0;
    std::vector<Color> colors = {
        {1, 0, 0},   // Red
        {0, 0, 1},   // Blue
        {1, 1, 0},   // Yellow
        {1, 0, 1},   // Magenta
        {0, 1, 1},   // Cyan
        {0.5, 0.5, 0.5}, // Gray
        {1, 0.5, 0}, // Orange
        {0, 0.5, 1}, // Light Blue
        {0.5, 0, 0.5} // Purple
    };

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        point_cloud_processor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

        Box box = point_cloud_processor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}