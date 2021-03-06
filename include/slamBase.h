#pragma once

#include <fstream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx,cy,fx,fy,scale;
};

// 函数接口
// 将rgb图转换为点云
PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, 
                                CAMERA_INTRINSIC_PARAMETERS &camera);

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input：3维点Point3f（u, v, d）
cv::Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera);

    

