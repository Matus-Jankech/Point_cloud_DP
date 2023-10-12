#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

class CloudHandler
{
public:
	CloudHandler();

	bool load_point_cloud(std::string file_name);
	void show_cloud();

private:
	static void keyboard_event_occurred(const pcl::visualization::KeyboardEvent& event, void* viewer);

private:
	std::string resource_path_ = "C:/Users/admin/Documents/Visual Studio 2022/Projects/Point_cloud_DP/Resources";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

};

