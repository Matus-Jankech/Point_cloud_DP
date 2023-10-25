#include <cloud_handler.h>

CloudHandler::CloudHandler()
{
}

void CloudHandler::filter_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, int meanK, double std_dev)
{	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	
	sor.setMeanK(meanK); // 50
	sor.setStddevMulThresh(std_dev); // 4
	
	std::cerr << "Filtering inliers... " << std::endl;
	sor.setInputCloud(input_cloud);
	sor.setNegative(false);
	sor.filter(*cloud_inliers);

	std::cerr << "Filtering outliers... " << std::endl;
	sor.setNegative(true);
	sor.filter(*cloud_outliers);
	
	save_cloud<pcl::PointXYZRGB>(cloud_inliers, "street_cloud_inliers.ply");
	save_cloud<pcl::PointXYZRGB>(cloud_outliers, "street_cloud_outliers.ply");
}

void CloudHandler::calculate_normals_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
												pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals_small,
												pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals_large)
{	
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;

	std::cerr << "Computing small normals... " << std::endl;
	ne.setInputCloud(input_cloud);
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.15); // Use all neighbors in a sphere of radius 5cm
	ne.compute(*cloud_normals_small);
	save_cloud<pcl::PointNormal>(cloud_normals_small, "normals_small.pcd");
	
	std::cerr << "Computing large normals... " << std::endl;
	ne.setInputCloud(input_cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.5); // Use all neighbors in a sphere of radius 50cm
	ne.compute(*cloud_normals_large);
	save_cloud<pcl::PointNormal>(cloud_normals_large, "normals_large.pcd");
}

void CloudHandler::calculate_don(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
								 pcl::PointCloud<pcl::PointNormal>::Ptr& don_cloud,
								 pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals_small,
								 pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals_large)
{
	pcl::copyPointCloud(*input_cloud, *don_cloud);

	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PointNormal> don;
	don.setInputCloud(input_cloud);
	don.setNormalScaleLarge(cloud_normals_large);
	don.setNormalScaleSmall(cloud_normals_small);

	if (!don.initCompute())
	{
		std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
		exit(EXIT_FAILURE);
	}

	std::cout << "Calculating DoN... " << std::endl;
	don.computeFeature(*don_cloud);
	save_cloud<pcl::PointNormal>(don_cloud, "don.pcd");
}

void CloudHandler::DoN_based_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{	
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_small(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_large(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr don_cloud(new pcl::PointCloud<pcl::PointNormal>);

	// Get normlas estimation
	if (load_cloud<pcl::PointNormal>(cloud_normals_small, "normals_small.pcd") == false ||
		load_cloud<pcl::PointNormal>(cloud_normals_large, "normals_large.pcd") == false)
	{	
		std::cerr << "Could not load normals estimation, calculating new one..." << std::endl;
		calculate_normals_estimation(input_cloud, cloud_normals_small, cloud_normals_large);	
	}

	// Get DoN
	if (load_cloud<pcl::PointNormal>(don_cloud, "don.pcd") == false)
	{
		std::cerr << "Could not load difference of normals (DoN), calculating new one..." << std::endl;
		calculate_don(input_cloud, don_cloud, cloud_normals_small, cloud_normals_large);
	}

	// Build the filter
	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	std::vector<double> lower_threshold = { 0.0, 0.05, 0.1, 0.15, 0.2 };
	std::vector<double> upper_threshold = { 0.05, 0.1, 0.15, 0.2, 1.0 };

	
	for (int i = 0; i < lower_threshold.size(); i++)
	{	
		pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointNormal>());
		range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
			new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GE, lower_threshold[i])));
		range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
			new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::LT, upper_threshold[i])));

		pcl::ConditionalRemoval<pcl::PointNormal> condrem;
		condrem.setInputCloud(don_cloud);
		condrem.setCondition(range_cond);
		std::cout << "Filtering class " << i << "..." << std::endl;
		condrem.filter(*doncloud_filtered);

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
		kdtree.setInputCloud(input_cloud);

		//Iterate through the points in doncloud_filtered and find their indices in the main cloud
		for (size_t j = 0; j < doncloud_filtered->size(); ++j) {
			pcl::PointXYZRGB search_point;
			search_point.x = doncloud_filtered->points[j].x;
			search_point.y = doncloud_filtered->points[j].y;
			search_point.z = doncloud_filtered->points[j].z;

			std::vector<int> point_indices(1);
			std::vector<float> point_distances(1);

			if (kdtree.nearestKSearch(search_point, 1, point_indices, point_distances) > 0) {
				// Add the index of the nearest point to inliers
				inliers->indices.push_back(point_indices[0]);
			}
		}

		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(input_cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_filtered);

		std::string file_name = "street_classified_" + std::to_string(i) + ".ply";
		save_cloud<pcl::PointXYZRGB>(cloud_filtered, file_name);
	}
}

void CloudHandler::set_cloud_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, int r, int g, int b)
{
	for (size_t i = 0; i < input_cloud->points.size(); ++i) {
		input_cloud->points[i].r = r;
		input_cloud->points[i].g = g;
		input_cloud->points[i].b = b;
	}
}

void CloudHandler::show_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addPointCloud(input_cloud);

	PCL_INFO("Point cloud visualized \n");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

void CloudHandler::show_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr& input_cloud)
{
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addPointCloudNormals<pcl::PointNormal>(input_cloud, 1, 1, "normals");

	PCL_INFO("Point cloud visualized \n");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

void CloudHandler::show_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& input_clouds,
							   std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& input_normals)
{
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

	for (size_t i = 0; i < input_clouds.size(); i++) {
		std::string cloud_id = "cloud_" + std::to_string(i);
		viewer.addPointCloud(input_clouds[i], cloud_id);
	}

	for (size_t i = 0; i < input_normals.size(); i++) {
		std::string normals_id = "normals_" + std::to_string(i);
		viewer.addPointCloudNormals<pcl::PointNormal>(input_normals[i], 1, 1, normals_id);
	}
	
	PCL_INFO("Point cloud visualized \n");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

