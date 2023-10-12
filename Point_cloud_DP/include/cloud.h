#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

class Cloud
{
public:
	Cloud();

	bool load_cloud(std::string file_name);
	bool save_cloud(std::string file_name);
	void show_cloud();
	void filter_outliers();
	void set_cloud_color(int r, int g, int b);

public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

private:
	static void keyboard_event_occurred(const pcl::visualization::KeyboardEvent& event, void* viewer);

private:
	std::string resource_path_ = "C:/Users/admin/Documents/Visual Studio 2022/Projects/Point_cloud_DP/Resources";

};

