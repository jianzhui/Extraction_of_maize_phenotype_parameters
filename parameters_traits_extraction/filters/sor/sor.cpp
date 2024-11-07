#include <iostream>

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

#include <pcl/filters/statistical_outlier_removal.h>


int

main ()

{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file/home/jianzhui/pcl-codes/datasets/maize2/5/con/000..pcd
  // reader.read<pcl::PointXYZ> ("/home/jianzhui/pcl-codes/datasets/maize2/8/con/240.pcd", *cloud);
  reader.read<pcl::PointXYZ> ("/home/jianzhui/pcl-codes/datasets/maize/icp/8.pcd", *cloud);
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cout << cloud->size() << std::endl;
  // std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (20);
  sor.setStddevMulThresh (1);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cout << "point size: " <<  cloud_filtered->size () << std::endl;
  // std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("after.pcd", *cloud_filtered, false);
  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("after_out.pcd", *cloud_filtered, false);

  return (0);
}
