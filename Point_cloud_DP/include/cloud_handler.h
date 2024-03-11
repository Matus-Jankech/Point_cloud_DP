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
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/poisson.h>
#include <pcl/common/distances.h>
#include <pcl/Vertices.h>

#include <execution>
#include <vector>
#include <algorithm>
#include <unordered_set>

#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/supervoxel_clustering.h>
#include <vtkPolyLine.h>
#include <pcl/segmentation/cpc_segmentation.h>
#include <filesystem>

#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>

#include <pcl/surface/texture_mapping.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <pcl/io/obj_io.h>
#include <pcl/common/transforms.h>

class CloudHandler
{
public:
	CloudHandler();

	// Visualization, Saving/Loading, Utilities
	template <typename PointType>
	bool load_cloud(typename pcl::PointCloud<PointType>::Ptr& cloud, std::string file_name);
	template <typename PointType>
	bool save_cloud(typename pcl::PointCloud<PointType>::Ptr& cloud, std::string file_name);
	void show_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);
	void show_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr& input_cloud);
	void show_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& input_clouds = std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(),
					 std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& input_normals = std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>());
	void set_cloud_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, int r, int g, int b);
	bool load_mesh(pcl::PolygonMesh::Ptr& input_mesh, std::string file_name);
	void show_mesh(pcl::PolygonMesh::Ptr& input_mesh);
	inline void set_base_path(std::string base) { resource_path_ = resource_path_ + "/" + base; }
	int find_max_object_index();
	void show_mesh_cloud(pcl::PolygonMesh::Ptr& input_mesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);
	bool load_textured_mesh(pcl::TextureMesh::Ptr& input_mesh, std::string file_name);
	void show_textured_mesh(pcl::TextureMesh::Ptr& input_mesh);

	// Filtering
	void filter_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, int meanK = 50, double std_dev = 4);
	void filter_ground_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);
	void downsample_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, float leaf_size, std::string file_name);
	void downsample_objects();
	void adaptive_downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, double lower_limit, double upper_limit, std::string file_name);

	// Segmentation
	void cpc_segmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input_cloud, bool visualize = false);

	// Mesh
	void create_mesh_GPT(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, std::string file_name);
	void create_mesh_Poison(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, 
							pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_full,
							std::string file_name, int depth);
	void create_mesh_MC(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, std::string file_name);
	void create_mesh_objects();
	void combine_mesh_ground_objects();

	// Texturing
	void texturize_mesh(pcl::PolygonMesh::Ptr& input_mesh);

private:
	void calculate_normals_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
									  pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals, double limit );
	void calculate_don(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
					   pcl::PointCloud<pcl::PointNormal>::Ptr& don_cloud,
					   pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals_small,
					   pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals_large);
	void separate_segmented_clouds(bool visualize = false);
	void downsample_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& input_clouds, std::string file_name);
	int saveOBJFile(const std::string& file_name, const pcl::TextureMesh& tex_mesh, unsigned precision);
	std::ifstream& GotoLine(std::ifstream& file, unsigned int num);
	bool readCamPoseFile(std::string filename, pcl::TextureMapping<pcl::PointXYZ>::Camera& cam);
	void showCameras(pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
	bool isFaceProjected(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3, pcl::PointXY& proj1, pcl::PointXY& proj2, pcl::PointXY& proj3);
	bool getPointUVCoordinates(const pcl::PointXYZ& pt, pcl::PointXY& UV_coordinates);
	void getTriangleCircumcscribedCircleCentroid(const pcl::PointXY& p1, const pcl::PointXY& p2, const pcl::PointXY& p3, pcl::PointXY& circumcenter, double& radius);
	bool checkPointInsideTriangle(const pcl::PointXY& p1, const pcl::PointXY& p2, const pcl::PointXY& p3, const pcl::PointXY& pt);
	void map_textures_on_mesh(pcl::TextureMesh& mesh, const pcl::texture_mapping::CameraVector& cameras);

public:
	std::string resource_path_ = "C:/Users/admin/Documents/Visual Studio 2022/Projects/Point_cloud_DP/Resources";
	std::vector<std::array<int, 3>> segment_colors_ = { {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {128, 0, 0}, {0, 128, 0}, {0, 0, 128}, {0, 128, 128}, {128, 128, 0}, {128, 0, 128}, {0, 128, 128}, {128, 128, 128}, {255, 165, 0}, {128, 0, 0}, {128, 128, 0}, {0, 128, 0}, {128, 128, 128}, {165, 42, 42}, {255, 192, 203}, {255, 20, 147}, {0, 255, 127}, {139, 0, 139}, {255, 255, 255}, {0, 255, 255}, {255, 140, 0}, {255, 255, 0}, {106, 90, 205}, {255, 69, 0}, {0, 250, 154} };

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