#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>

void saveColorPointsToPCD(const std::string& filename, const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>& cloud_ref = const_cast<pcl::PointCloud<pcl::PointXYZRGB>&>(cloud);  // 解除 const 限制
    cloud_ref.width = cloud_ref.points.size();
    cloud_ref.height = 1;  // 表示这是一个无序点云
    cloud_ref.is_dense = true;

    if (pcl::io::savePCDFileASCII(filename, cloud_ref) == -1) {
        std::cerr << "Error saving PCD file: " << filename << std::endl;
    } else {
        std::cout << "Saved " << cloud_ref.points.size() << " points to " << filename << std::endl;
    }
}

void colorSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr segment, pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud, int color_idx) {
    // 定义颜色 (红, 绿, 蓝)
    std::vector<std::vector<int>> colors = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}};

    // 遍历每个点并赋予颜色
    for (const auto& point : segment->points) {
        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;

        // 根据当前 color_idx 赋予颜色
        colored_point.r = colors[color_idx][0];
        colored_point.g = colors[color_idx][1];
        colored_point.b = colors[color_idx][2];

        // 将有颜色的点添加到输出点云中
        colored_cloud->points.push_back(colored_point);
    }
}

// 保存点到 PCD 文件的函数
void savePointsToPCD(const std::vector<pcl::PointXYZ>& path_points, const pcl::PointXYZ& point_start, const pcl::PointXYZ& point_end, const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 添加起点
    cloud->points.push_back(point_start);

    // 添加路径点
    for (const auto& point : path_points) {
        cloud->points.push_back(point);
    }

    // 添加终点
    cloud->points.push_back(point_end);

    // 设置点云的宽度和高度
    cloud->width = cloud->points.size();
    cloud->height = 1;  // 表示这是一个无序点云
    cloud->is_dense = true;

    // 保存到 PCD 文件
    pcl::io::savePCDFileASCII(filename, *cloud);
    std::cout << "Saved " << cloud->points.size() << " points to " << filename << std::endl;
}

// 计算两点间的欧氏距离
double computeEuclideanDistance(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2) {
    return std::sqrt(
        std::pow(point1.x - point2.x, 2) +
        std::pow(point1.y - point2.y, 2) +
        std::pow(point1.z - point2.z, 2)
    );
}

// 找到叶片上最远的两点
std::pair<pcl::PointXYZ, pcl::PointXYZ> findFurthestPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr leaf_cloud) {
    double max_distance = 0.0;
    pcl::PointXYZ p1, p2;

    for (size_t i = 0; i < leaf_cloud->points.size(); ++i) {
        for (size_t j = i + 1; j < leaf_cloud->points.size(); ++j) {
            double distance = computeEuclideanDistance(leaf_cloud->points[i], leaf_cloud->points[j]);
            if (distance > max_distance) {
                max_distance = distance;
                p1 = leaf_cloud->points[i];
                p2 = leaf_cloud->points[j];
            }
        }
    }
    return std::make_pair(p1, p2);
}

// 计算叶片长度：沿x轴切割，并选择代表点
// 计算叶片长度：沿x轴切割，并选择代表点
// 计算叶片长度：沿x轴切割，并选择代表点
double computeLeafLength(pcl::PointCloud<pcl::PointXYZ>::Ptr leaf_cloud, double step_size, std::vector<pcl::PointXYZ>& path_points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud) {
    // 1. 找到叶片上最远的两点，作为初始点
    auto furthest_points = findFurthestPoints(leaf_cloud);
    pcl::PointXYZ point_start = furthest_points.first;
    pcl::PointXYZ point_end = furthest_points.second;

    double start;
    double  end;
    // 2. 判断切割方向
    if(point_start.x < point_end.x)
    {
        start = point_start.x;
        end = point_end.x;
    }
    else
    {
        start = point_end.x;
        end =  point_start.x;
    }
    // 3. 沿 x 轴或根据最远两点的方向切割点云
    double leaf_length = 0.0;
    pcl::PassThrough<pcl::PointXYZ> pass;

    //used to judge red, green, blue
    int color_idx = 0;

    while(start < end) {
        // 设置过滤器沿 x 轴切割
        pass.setInputCloud(leaf_cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(start, start+ step_size);
        std::cout << " enter : "<< std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr segment(new pcl::PointCloud<pcl::PointXYZ>());
        pass.filter(*segment);

        // 检查是否切割到有效的点
        if (!segment->points.empty()) {
            //store the point cloud with color.
            colorSegment(segment, colored_cloud, color_idx);
            color_idx = (color_idx + 1) % 3;
            // 4. 计算每段点云的质心作为代表点
            pcl::PointXYZ centroid;
            centroid.x = 0.0;
            centroid.y = 0.0;
            centroid.z = 0.0;

            for (const auto& point : segment->points) {
                centroid.x += point.x;
                centroid.y += point.y;
                centroid.z += point.z;
            }

            // 计算质心
            centroid.x /= segment->points.size();
            centroid.y /= segment->points.size();
            centroid.z /= segment->points.size();

            // 将质心作为该段的代表点
            path_points.push_back(centroid);

            // 如果已经有路径点，则计算它们之间的距离
            if (path_points.size() > 1) {
                double tmp = computeEuclideanDistance(path_points[path_points.size() - 2], path_points.back());
                std::cout << "tmp: " << tmp << std::endl;
                leaf_length += tmp;
            }
        }

        // 更新 
        start += step_size;
    }

    std::cout << "leaf_length: "<< leaf_length << std::endl;
    // add the start point with point.front, end_point with point
    if (path_points.size() != 0) {
        if(point_start.x < point_end.x)
        {
        leaf_length += computeEuclideanDistance(path_points.front(), point_start);
        leaf_length += computeEuclideanDistance(path_points.back(), point_end);
        }
        else
        {
        leaf_length += computeEuclideanDistance(path_points.back(), point_start);
        leaf_length += computeEuclideanDistance(path_points.front(), point_end);
        }
    }

    return leaf_length;
}



int main(int argc, char** argv) {
    if(argc !=2 )
    {
        std::cerr << "Usage: " << argv[0] << "<pcd_file" << std::endl;
        return -1;
    }

    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr leaf_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile(argv[1], *leaf_cloud);   

    // 找到叶片最远的两点
    auto furthest_points = findFurthestPoints(leaf_cloud);
    pcl::PointXYZ point_start = furthest_points.first;
    pcl::PointXYZ point_end = furthest_points.second;
    // std::cout << "x : "<< point_start.x << std::endl;
    double distance = computeEuclideanDistance(point_start, point_end);
 
    // 设置步长，用于切割叶片
    double step_size = distance / 50;  // 你可以调整步长
    std::cout << "size : "<< step_size << std::endl;
    std::vector<pcl::PointXYZ> path_points;

    //create a color point cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // 计算叶长，并记录各个路径点
    double leaf_length = computeLeafLength(leaf_cloud, step_size, path_points, colored_cloud);
    std::cout << "Leaf length: " << leaf_length << " meters." << std::endl;

    std::string filename = "leaf_point.pcd";
    savePointsToPCD(path_points, point_start, point_end, filename);
    saveColorPointsToPCD("color.pcd", *colored_cloud);
    // 可视化叶片、路径点、起点和终点
    // visualizeLeaf(leaf_cloud, path_points, point_start, point_end);

    return 0;
}


