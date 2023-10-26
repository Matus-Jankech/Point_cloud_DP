#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>

class CloudHandler
{
public:
	CloudHandler();

	// Utilities
	template <typename PointType>
	bool load_cloud(typename pcl::PointCloud<PointType>::Ptr& cloud, std::string file_name);
	template <typename PointType>
	bool save_cloud(typename pcl::PointCloud<PointType>::Ptr& cloud, std::string file_name);

	void show_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);
	void show_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr& input_cloud);
	void show_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& input_clouds = std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(),
					 std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& input_normals = std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>());
	void set_cloud_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, int r, int g, int b);

	// Filtering
	void filter_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, int meanK = 50, double std_dev = 4);
	void DoN_based_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, double lower_limit, double upper_limit);

private:
	void calculate_normals_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
									  pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals, double limit );
	void calculate_don(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
					   pcl::PointCloud<pcl::PointNormal>::Ptr& don_cloud,
					   pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals_small,
					   pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals_large);

private:
	std::string resource_path_ = "C:/Users/admin/Documents/Visual Studio 2022/Projects/Point_cloud_DP/Resources";

};


/* -------------------------------------------------------
Templates must be in .hpp file because of linking problems
------------------------------------------------------- */

// Point cloud loading
template <typename PointType>
bool CloudHandler::load_cloud(typename pcl::PointCloud<PointType>::Ptr& cloud, std::string file_name)
{
	std::string cloud_type = file_name.substr(file_name.length() - 3);

	if (cloud_type == "ply")
	{
		PCL_INFO("Loading point cloud %s \n", file_name.c_str());
		if (pcl::io::loadPLYFile<PointType>(resource_path_ + "/" + file_name, *cloud) == -1)
		{
			PCL_ERROR("Couldn't read file: %s \n", file_name.c_str());
			return false;
		}
		PCL_INFO("Loaded %d data points from: %s \n", cloud->width * cloud->height, file_name.c_str());
		return true;
	}
	else if (cloud_type == "pcd")
	{
		PCL_INFO("Loading point cloud %s \n", file_name.c_str());
		if (pcl::io::loadPCDFile<PointType>(resource_path_ + "/" + file_name, *cloud) == -1)
		{
			PCL_ERROR("Couldn't read file: %s \n", file_name.c_str());
			return false;
		}
		PCL_INFO("Loaded %d data points from: %s \n", cloud->width * cloud->height, file_name.c_str());
		return true;
	}
	else
	{
		PCL_ERROR("Unknown point cloud type \n");
		return false;
	}
}

// Point cloud saving
template <typename PointType>
bool CloudHandler::save_cloud(typename pcl::PointCloud<PointType>::Ptr& cloud, std::string file_name)
{
	std::string cloud_type = file_name.substr(file_name.length() - 3);

	if (cloud_type == "ply")
	{
		pcl::PLYWriter writer;
		PCL_INFO("Writing file: cloud %s \n", file_name.c_str());
		if (writer.write<PointType>(resource_path_ + "/" + file_name, *cloud, true, false) == -1)
		{
			PCL_ERROR("Couldn't save file: %s \n", file_name.c_str());
			return false;
		}
		PCL_INFO("Saved file: %s \n", file_name.c_str());
		return true;
	}
	else if (cloud_type == "pcd")
	{	
		pcl::PCDWriter writer;
		PCL_INFO("Writing file: cloud %s \n", file_name.c_str());
		if (writer.write<PointType>(resource_path_ + "/" + file_name, *cloud, true) == -1)
		{
			PCL_ERROR("Couldn't save file: %s \n", file_name.c_str());
			return false;
		}
		PCL_INFO("Saved file: %s \n", file_name.c_str());
		return true;
	}
	else
	{
		PCL_ERROR("Unknown point cloud type \n");
		return false;
	}
}