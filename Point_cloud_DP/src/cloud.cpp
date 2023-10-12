#include <cloud.h>

Cloud::Cloud() :
	cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
{

}

bool Cloud::load_cloud(std::string file_name)
{
	PCL_INFO("Loading point cloud %s \n",file_name.c_str());
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(resource_path_ + "/" + file_name, *cloud_) == -1)
	{
		PCL_ERROR("Couldn't read file %s \n", file_name.c_str());
		return (false);
	}
	PCL_INFO("Loaded %d data points from %s \n", cloud_->width*cloud_->height, file_name.c_str());
	return true;
}

bool Cloud::save_cloud(std::string file_name)
{
	pcl::PLYWriter writer;
	if (writer.write<pcl::PointXYZRGB>(file_name, *cloud_, true) == -1)
	{
		PCL_ERROR("Couldn't save file %s \n", file_name.c_str());
		return (false);
	}
	PCL_INFO("Saved file %s \n", file_name.c_str());
	return true;
}

void Cloud::filter_outliers()
{	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud_);
	sor.setMeanK(25);
	sor.setStddevMulThresh(5.0);
	
	// Outliers
	std::cerr << "Filtering outliers... " << std::endl;
	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	std::cerr << "Writting outliers... " << std::endl;
	cloud_ = cloud_filtered;
	save_cloud("street_cloud_outliers.ply");

	// Inliers
	std::cerr << "Filtering inliers... " << std::endl;
	sor.setNegative(false);
	sor.filter(*cloud_filtered);
	std::cerr << "Writting inliers... " << std::endl;
	cloud_ = cloud_filtered;
	save_cloud("street_cloud_inliers.ply");
}

void Cloud::set_cloud_color(int r, int g, int b)
{
	for (size_t i = 0; i < cloud_->points.size(); ++i) {
		cloud_->points[i].r = r; 
		cloud_->points[i].g = g; 
		cloud_->points[i].b = b; 
	}
}

/* Visualization */
void Cloud::show_cloud()
{
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud_);
	PCL_INFO("Point cloud visualized \n");
	while (!viewer.wasStopped()){}
}

