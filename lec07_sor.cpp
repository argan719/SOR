#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>

void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored, const std::vector<int> &color)
{
    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (const auto &pt : pc.points)
    {
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Load input PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ubuntu/catkin_ws/src/pcl_tutorial/materials/AI7FCorner.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read input PCD file.\n");
        return -1;
    }

    // Apply Statistical Outlier Removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(130); // Adjust these parameters as needed
    sor.setStddevMulThresh(0.00001);
    sor.filter(*cloud_filtered);

    // Save the original, inlier, and outlier PCD files
    // pcl::io::savePCDFile("/home/ubuntu/catkin_ws/src/pcl_tutorial/materials/AI7FCorner_outlier.pcd", *cloud);
    pcl::io::savePCDFile("/home/ubuntu/catkin_ws/src/pcl_tutorial/materials/AI7FCorner_inlier.pcd", *cloud_filtered);
    // sor.setNegative(true);
    // sor.filter(*cloud_filtered);
    // pcl::io::savePCDFile("/home/ubuntu/catkin_ws/src/pcl_tutorial/materials/AI7FCorner_outlier.pcd", *cloud_filtered);

    // Visualize the point clouds
    pcl::visualization::PCLVisualizer viewer("PointCloud Visualization");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colorize(*cloud, *colored_cloud, {255, 0, 0}); // Red for original
    viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "original_cloud");
    colorize(*cloud_filtered, *colored_cloud, {0, 255, 0}); // Green for filtered
    viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "filtered_cloud");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    // 2D Projection of the inlier PCD
    int width = 960, height = 840;
    double scale_factor = 6.0; // Keep or adjust the scale factor as needed

    cv::Mat projection_image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    int min_u = INT_MAX;
    int max_u = INT_MIN;
    int min_v = INT_MAX;
    int max_v = INT_MIN;

    // First, find the bounding box of the points to center them
    for (const auto &point : cloud_filtered->points)
    {
        int u = static_cast<int>(point.x * scale_factor);
        int v = static_cast<int>(point.z * scale_factor);

        if (u < min_u)
            min_u = u;
        if (u > max_u)
            max_u = u;
        if (v < min_v)
            min_v = v;
        if (v > max_v)
            max_v = v;
    }

    // Calculate the offsets based on the bounding box
    int u_offset = (width / 2) - ((min_u + max_u) / 2);
    int v_offset = (height / 2) - ((min_v + max_v) / 2);

    // Use the offsets to center the projection
    for (const auto &point : cloud_filtered->points)
    {
        int u = static_cast<int>(point.x * scale_factor + u_offset);
        int v = static_cast<int>(point.z * scale_factor + v_offset);

        if (u >= 0 && u < width && v >= 0 && v < height)
        {
            projection_image.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 0);
        }
    }

    cv::imshow("2D Projection", projection_image);
    cv::waitKey(0);
    cv::imwrite("/home/ubuntu/catkin_ws/src/pcl_tutorial/materials/2d_projection_centered.png", projection_image);

    return 0;
}