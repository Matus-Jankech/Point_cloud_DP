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
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/time.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/segmentation/extract_clusters.h>

class Cloud
{
public:
	Cloud();

	bool load_cloud(std::string file_name);
	bool save_cloud(std::string file_name);
	void set_cloud_color(int r, int g, int b);
	void show_cloud();

	void normal_estimation();
	void ransac_ground_estimation();
	void normal_based_segmentation();

	void filter_outlier_points();
	void filter_ground_points(double lower_limit, int upper_limit);

public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_large_;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_small_;

private:
	std::string resource_path_ = "C:/Users/admin/Documents/Visual Studio 2022/Projects/Point_cloud_DP/Resources";

};

