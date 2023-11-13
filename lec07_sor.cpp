#include <iostream>
#include<cstring>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#//include "opencv2/core.hpp"



void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored, const std::vector<int> &color) {
    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (const auto &pt : pc.points) {
        pt_tmp.x = pt.x; pt_tmp.y = pt.y; pt_tmp.z = pt.z;
        pt_tmp.r = color[0]; pt_tmp.g = color[1]; pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    std::string path = "/home/jihyun/sor/gachon_hall.pcd";
    // Load input PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
        PCL_ERROR("Couldn't read input PCD file.\n");
        return -1;
    }

    // Apply Statistical Outlier Removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);  // Adjust these parameters as needed
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    // Save the original, inlier, and outlier PCD files
    pcl::io::savePCDFile(path, *cloud);
    pcl::io::savePCDFile(path, *cloud_filtered);
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    pcl::io::savePCDFile(path, *cloud_filtered);

    // Visualize the point clouds
    pcl::visualization::PCLVisualizer viewer("PointCloud Visualization");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colorize(*cloud, *colored_cloud, {255, 0, 0}); // Red for original
    viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "original_cloud");
    colorize(*cloud_filtered, *colored_cloud, {0, 255, 0}); // Green for filtered
    viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "filtered_cloud");
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    // 2D Projection of the inlier PCD
    int width = 1280, height = 960;
    double scale_factor = 10.0;
    cv::Mat projection_image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    for (const auto& point : cloud_filtered->points) {
        int u = static_cast<int>(point.x * scale_factor + width / 2.0);
        int v = static_cast<int>(point.z * scale_factor + height / 2.0);
        if (u >= 0 && u < width && v >= 0 && v < height) {
            projection_image.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 0);
        }
    }
    
    std::string output = "/home/jihyun/sor/result/gachon_hall.png";
    cv::imshow("2D Projection", projection_image);
    cv::waitKey(0);
    cv::imwrite(output, projection_image);

    return 0;
}
