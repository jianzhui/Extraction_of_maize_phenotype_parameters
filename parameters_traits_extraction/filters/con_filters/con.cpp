#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>


int main (int argc, char** argv)

{
  //read the cloud data.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  
  // cloud->width  = 20;
  // cloud->height = 1;
  // cloud->resize (cloud->width * cloud->height);
  // pcl::PointXYZ point;
  // for(int i = -10; i < 10; i ++) 
  // {
  //   point.x = i;
  //   point.y = i;
  //   point.z = i;
  //   cloud->push_back(point);
  // }
  //pcl::io::savePCDFileASCII("before.pcd", *cloud); // save the file after filter.
  // std::string filename = "/home/jianzhui/pcl-codes/datasets/maize2/8/orgin/240.pcd";
  std::string filename = "/home/jianzhui/pcl-codes/datasets/maize/maize3/icp/1.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)//*打开点云文件    
  {
    PCL_ERROR("Couldn't read file test_pcd.pcd\n");//输出错误原因：读取文件失败
    return(-1);//程序结束
  }
  //std::cout << "cloud size: " << cloud->size() << std::endl;
 std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
  // build the condition
  pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionOr<pcl::PointXYZ> ());
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 0.14)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, 0.18)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, -0.2)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -0.1)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, -0.7)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -0.6)));
  
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (cloud);
  // condrem.setKeepOrganized(true);

  // apply filter
  condrem.filter (*cloud_filtered);
  //std::cout << "cloud size: " << cloud_filtered->size() << std::endl;
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::io::savePCDFileASCII("after.pcd", *cloud_filtered); // save the file after filter.
  // condrem.setNegative (true);
  // condrem.filter (*cloud_filtered);
  pcl::io::savePCDFileASCII("after_out.pcd", *cloud_filtered);
  return (0);
}
