#include <cloud.h>

Cloud::Cloud() :
	cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
	cloud_normals_large_(new pcl::PointCloud<pcl::PointNormal>),
	cloud_normals_small_(new pcl::PointCloud<pcl::PointNormal>)
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
	if (writer.write<pcl::PointXYZRGB>(file_name, *cloud_, true, false) == -1)
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
	sor.setMeanK(50);
	sor.setStddevMulThresh(4);
	
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
	morph_fil.setSlope(3);
	morph_fil.setInitialDistance(0.13);
	morph_fil.setMaxDistance(0.3);
	morph_fil.setNumberOfThreads(8);

	// Create the pass filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(filtered_data);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(lower_limit, upper_limit);

	pcl::ScopeTime scopeTime("PMF");
	{
		std::cerr << "Extracting ground indicies... " << std::endl;
		morph_fil.extract(ground->indices);
	}

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

void Cloud::ransac_ground_estimation()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.5);
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud(cloud_);
	pcl::ScopeTime scopeTime("RANSAC");
	{	
		std::cerr << "RANSAC... " << std::endl;
		seg.segment(*inliers, *coefficients);
	}
	if (inliers->indices.size() == 0)
	{
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		return;
	}

	// Extract the inliers
	std::cerr << "Filtering ground... " << std::endl;
	extract.setInputCloud(cloud_);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_filtered);

	std::cerr << "Writting ground... " << std::endl;
	cloud_ = cloud_filtered;
	save_cloud("street_cloud_RANSAC_ground.ply");
}

void Cloud::normal_estimation()
{
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
	ne.setInputCloud(cloud_);
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.15); // Use all neighbors in a sphere of radius 5cm
	ne.compute(*cloud_normals_small_);

	ne.setInputCloud(cloud_);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.5); // Use all neighbors in a sphere of radius 50cm
	ne.compute(*cloud_normals_large_);
}

void Cloud::normal_based_segmentation()
{	
	normal_estimation();
	double threshold = 0.1;

	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud(*cloud_, *doncloud);

	std::cout << "Calculating DoN... " << std::endl;
	// Create DoN operator
	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PointNormal> don;
	don.setInputCloud(cloud_);
	don.setNormalScaleLarge(cloud_normals_large_);
	don.setNormalScaleSmall(cloud_normals_small_);

	if (!don.initCompute())
	{
		std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
		exit(EXIT_FAILURE);
	}

	// Compute DoN
	don.computeFeature(*doncloud);

	// Save DoN features
	pcl::PCDWriter writer;
	writer.write<pcl::PointNormal>("don.pcd", *doncloud, false);


	pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>());
	range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
		new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, threshold)));
	// Build the filter
	pcl::ConditionalRemoval<pcl::PointNormal> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(doncloud);

	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

	// Apply filter
	condrem.filter(*doncloud_filtered);
	writer.write<pcl::PointNormal>("doncloud_filtered.pcd", *doncloud_filtered, false);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud_);

	// Iterate through the points in doncloud_filtered and find their indices in the main cloud
	for (size_t i = 0; i < doncloud_filtered->size(); ++i) {
		pcl::PointXYZRGB search_point;
		search_point.x = doncloud_filtered->points[i].x;
		search_point.y = doncloud_filtered->points[i].y;
		search_point.z = doncloud_filtered->points[i].z;

		std::vector<int> point_indices(1);
		std::vector<float> point_distances(1);

		if (kdtree.nearestKSearch(search_point, 1, point_indices, point_distances) > 0) {
			// Add the index of the nearest point to inliers
			inliers->indices.push_back(point_indices[0]);
		}
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud_);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_filtered);

	cloud_ = cloud_filtered;
	save_cloud("street_filtered.ply");
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
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addPointCloud(cloud_);
	//viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal>(cloud_, cloud_normals_small_, 5, 0.1, "normals");
	//viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal>(cloud_, cloud_normals_large_, 5, 0.2, "normals");

	PCL_INFO("Point cloud visualized \n");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

