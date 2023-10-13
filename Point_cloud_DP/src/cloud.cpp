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

void Cloud::filter_outlier_points()
{	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Create the SOR filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	
	// SOR param
	sor.setMeanK(25);
	sor.setStddevMulThresh(5.2);
	
	// Inliers
	std::cerr << "Filtering inliers... " << std::endl;
	sor.setInputCloud(cloud_);
	sor.setNegative(false);
	sor.filter(*cloud_inliers);

	// Outliers
	std::cerr << "Filtering outliers... " << std::endl;
	sor.setNegative(true);
	sor.filter(*cloud_outliers);
	
	// Saving
	std::cerr << "Writting inliers... " << std::endl;
	cloud_ = cloud_inliers;
	save_cloud("street_cloud_inliers.ply");

	std::cerr << "Writting outliers... " << std::endl;
	cloud_ = cloud_outliers;
	save_cloud("street_cloud_outliers.ply");
}

void Cloud::filter_ground_points(double lower_limit, int upper_limit)
{	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_data(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_data(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_data(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_data_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointIndicesPtr ground(new pcl::PointIndices);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	// Create the morph filtering object
	pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZRGB> morph_fil;
	morph_fil.setInputCloud(cloud_);
	morph_fil.setCellSize(0.1);
	morph_fil.setMaxWindowSize(16);
	morph_fil.setSlope(6);
	morph_fil.setInitialDistance(0.15);
	morph_fil.setMaxDistance(0.3);
	morph_fil.setNumberOfThreads(6);

	// Create the pass filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(filtered_data);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(lower_limit, upper_limit);

	std::cerr << "Extracting ground indicies... " << std::endl;
	morph_fil.extract(ground->indices);

	// Extrach ground data with morph filter
	std::cerr << "Filtering ground... " << std::endl;
	extract.setInputCloud(cloud_);
	extract.setIndices(ground);
	extract.setNegative(false);
	extract.filter(*filtered_data);

	pass.setNegative (false);
	pass.filter(*ground_data);

	std::cerr << "Writting ground... " << std::endl;
	cloud_ = ground_data;
	save_cloud("street_cloud_ground.ply");

	// Extrach non-ground data
	std::cerr << "Filtering objects... " << std::endl;
	extract.setIndices(ground);
	extract.setNegative(true);
	extract.filter(*object_data);

	pass.setNegative (true);
	pass.filter(*object_data_filtered);

	std::cerr << "Writting objects... " << std::endl;
	*combined_cloud = *object_data + *object_data_filtered;
	cloud_ = combined_cloud;
	save_cloud("street_cloud_objects.ply");
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

