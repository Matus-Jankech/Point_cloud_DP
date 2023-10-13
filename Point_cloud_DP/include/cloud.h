#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

class Cloud
{
public:
	Cloud();

	bool load_cloud(std::string file_name);
	bool save_cloud(std::string file_name);
	void set_cloud_color(int r, int g, int b);
	void show_cloud();

	void filter_outlier_points();
	void filter_ground_points();

public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

private:
	std::string resource_path_ = "C:/Users/admin/Documents/Visual Studio 2022/Projects/Point_cloud_DP/Resources";

};

