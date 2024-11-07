#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

#include <cmath>

// 定义点结构体
struct Point {
    double x, y, z;
};

// 计算两个向量的点积
double dotProduct(const Point& v1, const Point& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

// 计算向量的长度
double magnitude(const Point& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// 计算直线P1P2与平面的夹角
double calculateAngleBetweenLineAndPlane(const Point& p1, const Point& p2, const Point& planeNormal) {
    // 计算直线P1P2的方向向量
    Point lineVector = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};

    // 计算线方向向量和平面法向量的点积
    double dot = dotProduct(lineVector, planeNormal);

    // 计算向量的模长
    double lineMagnitude = magnitude(lineVector);
    double normalMagnitude = magnitude(planeNormal);

    // 计算余弦值
    double cosTheta = dot / (lineMagnitude * normalMagnitude);

    // 确保cosTheta在[-1, 1]范围内，避免数值误差
    cosTheta = std::fmax(-1.0, std::fmin(1.0, cosTheta));

    // 计算方向向量与法向量之间的夹角（弧度）
    double thetaRad = std::acos(cosTheta);

    // 将弧度转换为角度
    double angleDeg = thetaRad * (180.0 / M_PI);

    // 计算直线与平面之间的夹角
    double angleBetweenLineAndPlane;
    if (angleDeg <= 90.0) {
        angleBetweenLineAndPlane = 90.0 - angleDeg;
    } else {
        angleBetweenLineAndPlane = angleDeg - 90.0;
    }

    return angleBetweenLineAndPlane;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <pcd_file>" << std::endl;
        return -1;
    }

    // Step 1: Load the projected PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", argv[1]);
        return -1;
    }

    // Step 2: Compute the maximum distance between points in the cloud
    double max_distance = 0.0;
    pcl::PointXYZ p1, p2;

    // Iterate through all points to find the maximum distance
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        for (size_t j = i + 1; j < cloud->points.size(); ++j) {
            double distance = sqrt(pow(cloud->points[i].x - cloud->points[j].x, 2) +
                                   pow(cloud->points[i].y - cloud->points[j].y, 2) +
                                   pow(cloud->points[i].z - cloud->points[j].z, 2));
            if (distance > max_distance) {
                max_distance = distance;
                p1 = cloud->points[i];
                p2 = cloud->points[j];
            }
        }
    }

    std::cout << "The maximum distance is " << max_distance << " between points ("
              << p1.x << ", " << p1.y << ", " << p1.z << ") and ("
              << p2.x << ", " << p2.y << ", " << p2.z << ")." << std::endl;
    // 定义点的坐标
    Point pp1 = {p1.x, p1.y, p1.z}; // 点P1的坐标
    Point pp2 = {p2.x, p2.y, p2.z};  // 点P2的坐标

    // 定义平面的法向量，例如：Y轴平面，其法向量为 (0, 1, 0)
    Point planeNormal = {0.0, 1.0, 0.0};

    // 计算直线与平面的夹角
    double angle = calculateAngleBetweenLineAndPlane(pp1, pp2, planeNormal);

    std::cout << "The angle between the line and the plane is: " << angle << " degrees" << std::endl;

    // Step 3: Visualize the point cloud and mark the points
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // Add the original point cloud to the viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 255, 255); // White color
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color_handler, "original cloud");

    // Mark the two points with maximum distance
    pcl::PointCloud<pcl::PointXYZ>::Ptr highlight_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    highlight_cloud->push_back(p1);
    highlight_cloud->push_back(p2);

    // Use the same point size for the highlighted points
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> highlight_color_handler(highlight_cloud, 255, 0, 0); // Red color
    viewer->addPointCloud<pcl::PointXYZ>(highlight_cloud, highlight_color_handler, "highlight cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "highlight cloud");

    // Add coordinate axes
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // Keep the viewer window open
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}
