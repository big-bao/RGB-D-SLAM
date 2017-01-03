// C++ 标准库
#include <iostream>
#include <string>

using namespace std;

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

// 主函数
int main(int argc, char **argv)
{
	// 图像矩阵
	cv::Mat rgb, depth;
	
	// CV::imread()来读取图像
	// rgb 图像是8UC3的彩色图像(8位uchar,3通道(即R,G,B三通道))
	rgb = cv::imread("../data/rgb.png");
	
	// depth 图像是16UC1的单通道图像，注意flag设置为-1，表示读取原始数据不做任何修改
	// 深度图为单通道，每像素由16bit组成(ushort)
	depth = cv::imread("../data/depth.png", -1);
	
	// 使用智能指针，创建空点云。该指针用完会自动释放。
	PointCloud::Ptr cloud (new PointCloud);
	
	// 遍历深度图
	for (int m = 0; m < depth.rows; m++)
	{
		for (int n = 0; n < depth.cols; n++)
		{
			// 获取深度图中(m,n)处的值
			ushort d = depth.ptr<ushort>(m)[n];
		
			// 若 d 不存在，则跳过此值
			if (0 == d)
				continue;
			
			// 若 d 存在，则向点云增加一个点
			PointT point;
			
			// 计算这个点的空间坐标
			point.z = double(d) / camera_factor;
			point.x = (n - camera_cx) * point.z / camera_fx;
			point.y = (m - camera_cy) * point.z / camera_fy;
			
			// 从 rgb 图像中获取它的颜色
			// rgb 是三通道的BGR格式图，所以按下面的顺序获取颜色
			point.b = rgb.ptr<uchar>(m)[n*3];
			point.g = rgb.ptr<uchar>(m)[n*3 + 1];
			point.r = rgb.ptr<uchar>(m)[n*3 + 2];
			
			// 把 point 加入到点云中
			cloud->points.push_back(point);
		}
	}
	
	// 设置并保存点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "Point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;
	pcl::io::savePCDFile("../data/pointcloud.pcd", *cloud);
	
	// 清除数据并退出
	cloud->points.clear();
	cout << "Point cloud saved." << endl;
	
	return 0;
}
























