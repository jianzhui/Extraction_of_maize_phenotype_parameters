#include <iostream>

#include <pcl/point_cloud.h> // for PointCloud

#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/filters/project_inliers.h>

#include <pcl/io/pcd_io.h>


int main (int argc, char** argv)
{
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

  // Create a set of planar coefficients with X=Y=0,Z=1

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

  coefficients->values.resize (4);

  coefficients->values[0] = 0;
  coefficients->values[1] = 1;

  coefficients->values[2] = 0;

  coefficients->values[3] = 0;


  // Create the filtering object

  pcl::ProjectInliers<pcl::PointXYZ> proj;

  proj.setModelType (pcl::SACMODEL_PLANE);

  proj.setInputCloud (cloud);

  proj.setModelCoefficients (coefficients);

  proj.filter (*cloud_projected);

  pcl::PCDWriter writer;

  writer.write<pcl::PointXYZ> ("result.pcd", *cloud_projected, false);

    // Extract Y coordinates
    double max_distance = 0.0;
    pcl::PointXYZ p1, p2;

    // Iterate through all points to find the maximum distance
    for (size_t i = 0; i < cloud_projected->points.size(); ++i) {
        for (size_t j = i + 1; j < cloud_projected->points.size(); ++j) {
            double distance = sqrt(pow(cloud_projected->points[i].x - cloud_projected->points[j].x, 2) +
                                   pow(cloud_projected->points[i].y - cloud_projected->points[j].y, 2) +
                                   pow(cloud_projected->points[i].z - cloud_projected->points[j].z, 2));
            if (distance > max_distance) {
                max_distance = distance;
                p1 = cloud_projected->points[i];
                p2 = cloud_projected->points[j];
            }
        }
    }

    std::cout << "The maximum distance is " << max_distance << " between points ("
              << p1.x << ", " << p1.y << ", " << p1.z << ") and ("
              << p2.x << ", " << p2.y << ", " << p2.z << ")." << std::endl;
  return (0);

}