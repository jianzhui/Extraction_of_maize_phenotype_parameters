#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>
#include <iomanip>

int main(int argc, char** argv) {
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    reader.read("/home/jianzhui/pcl-codes/segmetationn/datasets/1-8/stem/leaf.pcd", *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl;
    
    //Create the sor filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (30);
    sor.setStddevMulThresh (0.8);
    sor.filter (*cloud);

    // 创建KdTree对象用于欧几里得聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.01); // 设置聚类容忍度为2cm
    ec.setMinClusterSize(5);    // 设置最小聚类大小
    ec.setMaxClusterSize(25000);  // 设置最大聚类大小
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (const auto& cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;

        std::stringstream ss;
        ss << "cloud_cluster_" << std::setw(4) << std::setfill('0') << j << ".pcd";
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);

        j++;
    }

    return 0;
}




