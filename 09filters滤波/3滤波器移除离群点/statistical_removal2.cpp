/*
 * @Description: 使用statisticalOutlierRemoval滤波器移除离群点
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-20 11:37:27
 * @LastEditTime: 2020-10-20 11:56:46
 * @FilePath: /pcl-learning/09filters滤波/3滤波器移除离群点/statistical_removal.cpp
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  // 定义读取对象
  pcl::PCDReader reader;
  // 读取点云文件
  std::string filename;
  if (argv[1] == NULL)
  {
    filename = "../table_scene_lms400.pcd";
  }
  else
  {
    filename = argv[1];
  }
  reader.read(filename, *cloud); //读取点云到cloud中
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList(*cloud) << ").";
  // std::cerr << *cloud << std::endl;

  // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
  //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //创建滤波器对象
  sor.setInputCloud(cloud);                             //设置待滤波的点云
  sor.setMeanK(50);                                     //设置在进行统计时考虑查询点临近点数
  sor.setStddevMulThresh(1.0);                          //设置判断是否为离群点的阀值
  sor.filter(*cloud_filtered);                          //存储

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")    .";
  // std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  filename.replace(filename.find(".pcd"), 4, "_inliers.pcd");
  writer.write<pcl::PointXYZRGB>(filename, *cloud_filtered, false);
  //true：滤波结果取反，被过滤的点
  sor.setNegative(true);
  sor.filter(*cloud_filtered);
  filename.replace(filename.find("_inliers.pcd"), 12, "_outliers.pcd");
  writer.write<pcl::PointXYZRGB>(filename, *cloud_filtered, false);

  return (0);
}