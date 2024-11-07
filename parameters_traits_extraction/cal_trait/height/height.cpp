#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

int main(int argc, char** argv) {
    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        std::cerr << "Couldn't read file " << argv[1] << std::endl;
        return -1;
    }

    // Extract Y coordinates
    std::vector<float> y_coords;
    for (const auto& point : *cloud) {
        y_coords.push_back(point.y);
    }

    // Find min/max Y coordinates
    float min_y = *std::min_element(y_coords.begin(), y_coords.end());
    float max_y = *std::max_element(y_coords.begin(), y_coords.end());

    // Calculate distance
    float distance = max_y - min_y;

    std::cout << "Vertical distance between highest and lowest points along Y axis: " << distance << std::endl;

    return 0;
}

