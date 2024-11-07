#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

struct EllipseParams {
    double A, B, C, D, E, F;
};

// 拟合椭圆函数
EllipseParams fitEllipse(const std::vector<Eigen::Vector2d>& point) {
    int N = point.size();
    Eigen::MatrixXd D(N, 6);
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(N);

    for (int i = 0; i < N; ++i) {
        double x = point[i][0];
        double y = point[i][1];
        D(i, 0) = x * x;
        D(i, 1) = x * y;
        D(i, 2) = y * y;
        D(i, 3) = x;
        D(i, 4) = y;
        D(i, 5) = 1.0;
    }

    Eigen::MatrixXd S = D.transpose() * D;
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6);
    C(0, 2) = 2.0;
    C(1, 1) = -1.0;
    C(2, 0) = 2.0;

    Eigen::EigenSolver<Eigen::MatrixXd> es(S.inverse() * C);
    Eigen::VectorXd a = es.eigenvectors().real().col(0);

    EllipseParams params;
    std::cout<< a(0)<< " "<< a(1)<< " "<< a(2)<< " "
             << a(3)<< " "<< a(4)<<  " "<< a(5)<< " "<< std::endl;
    params.A = a(0);
    params.B = a(1);
    params.C = a(2);
    params.D = a(3);
    params.E = a(4);
    params.F = a(5);

    return params;
}

// 计算点到椭圆的距离
bool calculateDistance(const PointT& point, const EllipseParams& params) {
    double x = point.x;
    double y = point.z;
    double value = params.A * x * x + params.B * x * y + params.C * y * y + params.D * x + params.E * y + params.F;
    return value <= 0.00002;
}

int main(int argc, char** argv) {
    // 加载完整玉米的点云
    PointCloud::Ptr cloud(new PointCloud);
    if (pcl::io::loadPCDFile<PointT>("/home/jianzhui/pcl-codes/datasets/maize/maize2/icp/8.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read the PCD file\n");
        return -1;
    }
    
    //选取部分进行投影拟合茎秆圆柱
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // build the con filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    //选取茎秆-0.06到-0.045的部分进行投影拟合
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, 0.042)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 0.066)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -0.03)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, 0.03)));
    // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -0.595)));
    // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, -0.603)));
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    // apply filter
    condrem.filter (*cloud_filtered);
    //std::cout << "cloud size: " << cloud_filtered->size() << std::endl;
    pcl::io::savePCDFileASCII("cloud_filtered.pcd", *cloud_filtered); 
    //投影到xoz平面上
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); 
    coefficients->values.resize (4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 1;
    coefficients->values[2] = 0;   
    coefficients->values[3] = 0;
   
    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_filtered);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    
    //Create the sor filtering object
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud (cloud_projected);
    // sor.setMeanK (50);
    // sor.setStddevMulThresh (0.8);
    // sor.filter (*cloud_projected);

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("cloud_projected.pcd", *cloud_projected, false);
   

    //加载投影后的玉米点云：进行拟合圆柱
    PointCloud::Ptr cloud2(new PointCloud);
    if (pcl::io::loadPCDFile<PointT>("cloud_projected.pcd", *cloud2) == -1) {
        PCL_ERROR("Couldn't read the PCD file\n");
        return -1;
    }

    // 将点云数据转换为Eigen格式
    std::vector<Eigen::Vector2d> selectedPoints;
    for (const auto& point : cloud2->points) 
    {
        selectedPoints.emplace_back(point.x, point.z);
    }
   
    EllipseParams ellipseParams = fitEllipse(selectedPoints);

    // 分类点云数据
    PointCloud::Ptr stemCloud(new PointCloud);
    PointCloud::Ptr leafCloud(new PointCloud);
    // double threshold = 0.001; // 距离阈值

    for (const auto& point : cloud->points) {
        PointT p = point;
        p.y = 0; // 将z设置为0
        int flag = calculateDistance(p, ellipseParams);
        if (flag) {
            stemCloud->points.push_back(point);
        } else {
            // std::cout << "enter";
            leafCloud->points.push_back(point);
        }
    }
    stemCloud->width = stemCloud->points.size();
    stemCloud->height = 1;
    stemCloud->is_dense = true;

    leafCloud->width = leafCloud->points.size();
    leafCloud->height = 1;
    leafCloud->is_dense = true;
    // // 保存分类后的点云
    // sor.setMeanK (50);
    // sor.setStddevMulThresh (1);
    // sor.filter (*cloud_projected);
    // sor.setInputCloud(stemCloud);
    // sor.filter(*stemCloud);
    // sor.setInputCloud(leafCloud);
    // sor.filter(*leafCloud);
    pcl::io::savePCDFileASCII("stem.pcd", *stemCloud);
    pcl::io::savePCDFileASCII("leaf.pcd", *leafCloud);

    return 0;
}
