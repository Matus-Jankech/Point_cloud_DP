#include <cloud.h>

void filter_outliers()
{
	Cloud cloud;
	if (cloud.load_cloud("street_cloud.ply"))
	{
		cloud.filter_outlier_points();
	}
}

void combine_show_clouds(std::string name1, std::string name2)
{
	Cloud cloud1, cloud2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (cloud1.load_cloud(name1) && cloud2.load_cloud(name2))
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		//cloud1.set_cloud_color(255, 255, 255);
		cloud2.set_cloud_color(200, 200, 200);

		*cloud1.cloud_ += *cloud2.cloud_;
		cloud1.show_cloud();
	}
}

void show_cloud(std::string name)
{
	Cloud cloud;
	if (cloud.load_cloud(name))
	{	
		//cloud.normal_estimation();
		cloud.show_cloud();
	}
}

void filter_ground()
{
	Cloud cloud;
	if (cloud.load_cloud("street_cloud_inliers.ply"))
	{
		cloud.filter_ground_points(179.0, 182.0);
	}
}


void filter_ground2()
{
	Cloud cloud;
	if (cloud.load_cloud("street_cloud_inliers.ply"))
	{
		cloud.normal_based_segmentation();
	}
}

void load_cloud(std::string file_name)
{	
	std::string resource_path = "C:/Users/admin/Documents/Visual Studio 2022/Projects/Point_cloud_DP/Resources";
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	PCL_INFO("Loading point cloud %s \n", file_name.c_str());
	if (pcl::io::loadPCDFile<pcl::PointNormal>(resource_path + "/" + file_name, *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file %s \n", file_name.c_str());
		return;
	}
	PCL_INFO("Loaded %d data points from %s \n", cloud->width * cloud->height, file_name.c_str());

	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addPointCloudNormals<pcl::PointNormal>(cloud, 1, 1, "normals");

	PCL_INFO("Point cloud visualized \n");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return;
}

void binary_to_ascii()
{
	Cloud cloud;
	if (cloud.load_cloud("street_cloud.ply"))
	{
		pcl::PLYWriter writer;
		if (writer.write<pcl::PointXYZRGB>("street_cloud_ascii.ply", *cloud.cloud_, false) == -1)
		{
			PCL_ERROR("Couldn't save file %s \n","street_cloud_ascii.ply");
		}
		PCL_INFO("Saved file %s \n", "street_cloud_ascii.ply");
	}
}

int main(int argc, char** argv)
{	
	//combine_show_clouds("street_cloud_inliers.ply", "street_cloud_outliers.ply");
	//combine_show_clouds("street_cloud_objects.ply", "street_cloud_ground.ply");

	//filter_ground();
	//combine_show_clouds("street_cloud_objects.ply", "street_cloud_ground.ply");
	
	//filter_ground2();
	//show_cloud("street_cloud_inliers.ply");
	//load_cloud("don.pcd");
	//load_cloud("doncloud_filtered.pcd");	

	show_cloud("street_filtered.ply");

	//show_cloud_normals("street_cloud_inliers.ply");
	//inary_to_ascii();
	//filter_outliers();
	
	return (0);
}
