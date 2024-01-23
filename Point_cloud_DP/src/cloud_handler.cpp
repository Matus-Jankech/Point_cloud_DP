#include <cloud_handler.h>

CloudHandler::CloudHandler()
{
}

void CloudHandler::filter_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, int meanK, double std_dev)
{	
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*input_cloud, *cloud_inliers);
	pcl::copyPointCloud(*input_cloud, *cloud_outliers);

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

void CloudHandler::filter_ground_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_data(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_data(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_data(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_data_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointIndicesPtr ground(new pcl::PointIndices);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	// Create the pass filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(input_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(179, 182);

	pcl::copyPointCloud(*input_cloud, *filtered_data);
	pass.setNegative(false);
	pass.filter(*filtered_data);

	// Create the filtering object
	std::cerr << "Calculating morp filter..." << std::endl;
	pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZRGB> morph_fil;
	morph_fil.setInputCloud(filtered_data);
	morph_fil.setCellSize(0.1);
	morph_fil.setMaxWindowSize(16);
	morph_fil.setSlope(3);
	morph_fil.setInitialDistance(0.13);
	morph_fil.setMaxDistance(0.3);
	morph_fil.setNumberOfThreads(8);
	morph_fil.extract(ground->indices);

	// Extrach ground data
	pcl::copyPointCloud(*input_cloud, *ground_data);
	std::cerr << "Filtering ground... " << std::endl;
	extract.setInputCloud(filtered_data);
	extract.setIndices(ground);
	extract.setNegative(false);
	extract.filter(*ground_data);

	save_cloud<pcl::PointXYZRGB>(ground_data, "street_cloud_ground.ply");

	// Extrach object data
	pcl::copyPointCloud(*input_cloud, *object_data);
	std::cerr << "Filtering objects... " << std::endl;
	extract.setIndices(ground);
	extract.setNegative(true);
	extract.filter(*object_data);

	pcl::copyPointCloud(*input_cloud, *object_data_filtered);
	pass.setNegative(true);
	pass.filter(*object_data_filtered);
	*combined_cloud = *object_data + *object_data_filtered;

	save_cloud<pcl::PointXYZRGB>(combined_cloud, "street_cloud_objects.ply");
}

void CloudHandler::downsample_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, float leaf_size, std::string file_name)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*input_cloud, *cloud_downsampled);

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(input_cloud);
	sor.setLeafSize(leaf_size, leaf_size, leaf_size);
	sor.filter(*cloud_downsampled);
	save_cloud<pcl::PointXYZRGB>(cloud_downsampled, file_name);
}

double calculateAverageDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);

	double total_distance = 0.0;
	int total_pairs = 0;

	for (size_t i = 0; i < cloud->size(); ++i) {
		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);

		if (kdtree.nearestKSearch((*cloud)[i], 2, indices, sqr_distances) > 0) {
			// Exclude the point itself by taking the second closest point
			if (indices[1] != static_cast<int>(i)) {
				total_distance += sqrt(sqr_distances[1]); // sqrt to get actual distance
				total_pairs++;
			}
		}
	}

	if (total_pairs > 0) {
		return total_distance / static_cast<double>(total_pairs);
	}
	else {
		return 0.0; // Return 0 if no pairs found (or just one point)
	}
}

void CloudHandler::downsample_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& input_clouds, std::string file_name)
{	
	std::vector<double> voxel_sizes = { 0.12, 0.05, 0.0 };
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;

	unsigned int index = 0;
	for (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud : input_clouds)
	{	
		if(voxel_sizes[index] != 0.0)
		{
			pcl::copyPointCloud(*input_cloud, *cloud_downsampled);
			sor.setInputCloud(input_cloud);
			sor.setLeafSize(voxel_sizes[index], voxel_sizes[index], voxel_sizes[index]);
			sor.filter(*cloud_downsampled);
			*cloud_all = *cloud_all + *cloud_downsampled;
		}
		else {
			*cloud_all = *cloud_all + *input_cloud;
		}

		index++;
	}

	save_cloud<pcl::PointXYZRGB>(cloud_all, file_name);
}

void CloudHandler::calculate_normals_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
												pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals, double limit)
{	
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
	std::string normals_file = "normlas_" + std::to_string(limit) + ".pcd";

	ne.setInputCloud(input_cloud);
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(limit); // Use all neighbors in a sphere of radius 5cm
	ne.compute(*cloud_normals);
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
	//save_cloud<pcl::PointNormal>(don_cloud, "don.pcd");
}

int CloudHandler::find_max_object_index()
{
	std::string directory_path = resource_path_ + "/Objects";
	int max_object_number = -1;

	std::filesystem::directory_iterator dir_iter(directory_path);
	for (const auto& entry : dir_iter) {
		if (!entry.is_directory()) {
			std::string filename = entry.path().filename().string();

			if (filename.find("object_") == 0 && filename.find(".ply") != std::string::npos) {
				try {
					int object_num = std::stoi(filename.substr(7, filename.find(".ply") - 7));

					if (object_num > max_object_number) {
						max_object_number = object_num;
					}
				}
				catch (...) {
					std::cerr << "Error converting filename to number: " << filename << std::endl;
				}
			}
		}
	}

	return max_object_number;
}

void CloudHandler::downsample_objects()
{
	int max_object_index = find_max_object_index();
	max_object_index = max_object_index + 1;

	for (int i = 0; i < max_object_index; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::string in_name = "Objects/object_" + std::to_string(i) + ".ply";
		std::string out_name = "Objects/object_" + std::to_string(i) + "_downsampled.ply";

		std::cout << "\n";
		load_cloud<pcl::PointXYZRGB>(object, in_name);
		adaptive_downsampling(object, 0.03, 0.30, out_name);
	}
}

void CloudHandler::adaptive_downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, double lower_limit, double upper_limit, std::string file_name)
{	
	std::vector<double> lower_threshold = { 0.0, 0.15, 0.3};
	std::vector<double> upper_threshold = { 0.15, 0.3, 1};

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_small(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_large(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr don_cloud(new pcl::PointCloud<pcl::PointNormal>);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_downsample;

	// Get normlas estimation + DoN
	calculate_normals_estimation(input_cloud, cloud_normals_large, upper_limit);
	calculate_normals_estimation(input_cloud, cloud_normals_small, lower_limit);
	calculate_don(input_cloud, don_cloud, cloud_normals_small, cloud_normals_large);

	
	for (int i = 0; i < lower_threshold.size(); i++)
	{	
		// Build the filter
		pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
		pcl::copyPointCloud(*input_cloud, *doncloud_filtered);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::copyPointCloud(*input_cloud, *cloud_filtered);

		pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointNormal>());
		range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
			new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GE, lower_threshold[i])));
		range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
			new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::LT, upper_threshold[i])));

		pcl::ConditionalRemoval<pcl::PointNormal> condrem;
		condrem.setInputCloud(don_cloud);
		condrem.setCondition(range_cond);
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
		clouds_to_downsample.push_back(cloud_filtered);
	}

	downsample_clouds(clouds_to_downsample, file_name);
}

void CloudHandler::create_mesh_GPT(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, std::string file_name)
{
	// Normal estimation*
	PCL_INFO("Calculating normal estimation... \n");
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(input_cloud);
	n.setInputCloud(input_cloud);
	n.setSearchMethod(tree);
	n.setRadiusSearch(0.8);
	n.compute(*normals);

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*input_cloud, *normals, *cloud_with_normals);

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(1.20);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(600);
	gp3.setMaximumSurfaceAngle(M_PI / 2); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	PCL_INFO("Calculating mesh... \n");
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	PCL_INFO("Saving mesh... \n");
	pcl::io::savePolygonFileVTK(resource_path_ + "/" + file_name + ".vtk", triangles);
	PCL_INFO("Mesh saved \n");
}

void CloudHandler::create_mesh_Poison(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, std::string file_name)
{
	// Normal estimation*
	PCL_INFO("Calculating normal estimation... \n");
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(input_cloud);
	n.setInputCloud(input_cloud);
	n.setSearchMethod(tree);
	n.setRadiusSearch(0.5);
	n.compute(*normals);

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*input_cloud, *normals, *cloud_with_normals);

	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setInputCloud(cloud_with_normals); // Set your input point cloud
	poisson.setDepth(13); // Set the depth level of the Octree
	poisson.setSolverDivide(50);
	poisson.setIsoDivide(50);
	poisson.setThreads(8);

	// Perform Poisson surface reconstruction
	PCL_INFO("Calculating mesh... \n");
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	// Define your threshold distance
	const double distanceThreshold = 0.01; 

	pcl::PointCloud<pcl::PointXYZ>::Ptr meshPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *meshPointCloud);

	PCL_INFO("Removing bad vertices... \n");
	auto start_time = std::chrono::high_resolution_clock::now();
	std::vector<pcl::Vertices>& polygons = mesh.polygons;
	std::vector<pcl::Vertices> new_polygons;
	new_polygons.reserve(polygons.size());
	std::atomic<int> progressCounter(0);
	std::mutex mutex;

	// Create KD-Tree for the input cloud
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(input_cloud);

	std::for_each(std::execution::par, std::begin(polygons), std::end(polygons), [&](pcl::Vertices& polygon)
	{
		pcl::Vertices new_polygon;
		std::vector<uint32_t> newIndices;

		for (size_t j = 0; j < polygon.vertices.size(); ++j) {
			uint32_t vertexIndex = polygon.vertices[j];
			pcl::PointXYZ searchPoint = meshPointCloud->at(vertexIndex);

			std::vector<int> nearest_indices(1);
			std::vector<float> nearest_distances(1);

			if (kdtree.nearestKSearch(searchPoint, 1, nearest_indices, nearest_distances) > 0) {
				if (nearest_distances[0] <= distanceThreshold) {
					newIndices.emplace_back(vertexIndex);
				}
			}

			// Check if there are even 3 vertices to create triangle
			if (polygon.vertices.size() - (j + 1) + newIndices.size() <= 2) {
				break;
			}
		}

		if (!newIndices.empty() && newIndices.size() > 2) {
			for (const auto& index : newIndices) {
				new_polygon.vertices.emplace_back(index);
			}
			std::lock_guard<std::mutex> lock(mutex);
			new_polygons.emplace_back(new_polygon);
		}

		progressCounter++;
		{
			std::lock_guard<std::mutex> lock(mutex);
			std::cout << "\rProgress: " << std::fixed << std::setprecision(3) << progressCounter * 100.00 / polygons.size() << "%" << std::flush;
		}
	});

	{
		std::lock_guard<std::mutex> lock(mutex); 
		mesh.polygons = new_polygons;
	}

	PCL_INFO("\nSaving mesh... \n");
	pcl::io::savePolygonFileVTK(resource_path_ + "/" + file_name, mesh);
	PCL_INFO("Mesh saved \n");
}

void CloudHandler::create_mesh_objects()
{
	int max_object_index = find_max_object_index();
	max_object_index = max_object_index + 1;

	for (int i = 0; i < max_object_index; i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
		std::string in_name = "Objects/object_" + std::to_string(i) + "_downsampled.ply";
		std::string out_name = "Objects/object_" + std::to_string(i) + "_mesh.vtk";

		std::cout << "\n";
		load_cloud<pcl::PointXYZ>(object, in_name);
		create_mesh_Poison(object, out_name);
	}
}

void CloudHandler::cpc_segmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input_cloud, bool visualize)
{	
	float voxel_resolution = 0.1f;
	float seed_resolution = 1.0f;
	float color_importance = 0.1f;
	float spatial_importance = 0.2f;
	float normal_importance = 1.0f;

	pcl::SupervoxelClustering<pcl::PointXYZRGBA> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(false);
	super.setInputCloud(input_cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);

	std::map <std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;

	pcl::console::print_highlight("Extracting supervoxels!\n");
	super.extract(supervoxel_clusters);
	pcl::console::print_info("Found %d supervoxels\n", supervoxel_clusters.size());

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
	viewer->addPointCloud(voxel_centroid_cloud, "voxel centroids");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroids");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");

	pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();
	viewer->addPointCloud(labeled_voxel_cloud, "labeled voxels");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "labeled voxels");

	pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);

	pcl::console::print_highlight("Getting supervoxel adjacency\n");
	std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);

	if (visualize)
	{
		while (!viewer->wasStopped()){ viewer->spinOnce(100); }
	}

	pcl::PCLPointCloud2 input_pointcloud2;
	pcl::toPCLPointCloud2(*input_cloud, input_pointcloud2);

	int concavity_tolerance_threshold = 15;
	int max_cuts = 25;
	int cutting_min_segments = 300; // Min number of segments to be valid
	int min_segment_size = 150;
	int ransac_iterations = 3000;
	int k_factor = 0.0;
	float min_cut_score = 0.3;
	float smoothness_threshold = 0.1;

	bool use_sanity_criterion = false;
	bool use_local_constrain = true;
	bool use_directed_cutting = false;
	bool use_clean_cutting = true;

	PCL_INFO("Starting Segmentation\n");
	pcl::CPCSegmentation<pcl::PointXYZRGBA> cpc;
	cpc.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	cpc.setSanityCheck(use_sanity_criterion);
	cpc.setCutting(max_cuts, cutting_min_segments, min_cut_score, use_local_constrain, use_directed_cutting, use_clean_cutting);
	cpc.setRANSACIterations(ransac_iterations);
	cpc.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	cpc.setKFactor(k_factor);
	cpc.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	cpc.setMinSegmentSize(min_segment_size);
	cpc.segment();

	PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr cpc_labeled_cloud = sv_labeled_cloud->makeShared();
	cpc.relabelCloud(*cpc_labeled_cloud);
	pcl::LCCPSegmentation<pcl::PointXYZRGBA>::SupervoxelAdjacencyList sv_adjacency_list;
	cpc.getSVAdjacencyList(sv_adjacency_list);  // Needed for visualization

	/// Creating Colored Clouds and Output
	if (cpc_labeled_cloud->size() == input_cloud->size())
	{
		PCL_INFO("Saving output\n");

		if (pcl::getFieldIndex(input_pointcloud2, "label") >= 0)
			PCL_WARN("Input cloud already has a label field. It will be overwritten by the cpc segmentation output.\n");
		pcl::PCLPointCloud2 output_label_cloud2, output_concat_cloud2;
		pcl::toPCLPointCloud2(*cpc_labeled_cloud, output_label_cloud2);
		pcl::concatenateFields(input_pointcloud2, output_label_cloud2, output_concat_cloud2);
		pcl::io::savePCDFile(resource_path_ + "/" + "segment_cloud.pcd", output_concat_cloud2, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);
		separate_segmented_clouds(visualize);
	}
	else
	{
		PCL_ERROR("ERROR:: Sizes of input cloud and labeled supervoxel cloud do not match. No output is produced.\n");
	}
}

void CloudHandler::separate_segmented_clouds(bool visualize)
{
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
	load_cloud<pcl::PointXYZRGBL>(segmented_cloud, "/segment_cloud.pcd");

	std::map<int, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> labelCloudMap;

	for (const auto& point : *segmented_cloud) {
		int label = static_cast<int>(point.label);

		if (labelCloudMap.find(label) == labelCloudMap.end()) {
			labelCloudMap[label] = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>);
		}

		labelCloudMap[label]->push_back(point);
	}

	int colorIndex = 0;
	int total = 0;
	int num_objects = 0;
	int min_num_of_points = 150;

	std::string directory = resource_path_ + "/Objects/";
	if (!std::filesystem::exists(directory)) {
		std::filesystem::create_directory(directory);
	}

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Segmented cloud"));
	for (const auto& pair : labelCloudMap) {
		std::string label_name = "Label_" + std::to_string(pair.first);
		std::string object_name = "object_" + std::to_string(num_objects);

		std::array<int, 3> color = segment_colors_[colorIndex % segment_colors_.size()];
		if (pair.second->size() > min_num_of_points)
		{
			if (visualize)
			{
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBL> color_handler(pair.second, color[0], color[1], color[2]);
				viewer->addPointCloud(pair.second, color_handler, label_name);
			}

			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGBL>);
			object = pair.second;
			save_cloud<pcl::PointXYZRGBL>(object,"Objects/" + object_name + ".ply");

			total = total + pair.second->size();
			num_objects++;
			colorIndex++;
		}
	}
	std::cout << "Total objects: " << num_objects << " ,size: " << total << std::endl;

	if (visualize)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
		load_cloud<pcl::PointXYZRGB>(ground, "street_cloud_ground.ply");
		set_cloud_color(ground, 80, 80, 80);
		viewer->addPointCloud(ground, "ground");

		while (!viewer->wasStopped()) { viewer->spinOnce(100); }
	}
	
	return;
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
	unsigned int size = 0;

	for (size_t i = 0; i < input_clouds.size(); i++) {
		std::string cloud_id = "cloud_" + std::to_string(i);
		viewer.addPointCloud(input_clouds[i], cloud_id);
		size = size + input_clouds[i]->size();
	}

	for (size_t i = 0; i < input_normals.size(); i++) {
		std::string normals_id = "normals_" + std::to_string(i);
		viewer.addPointCloudNormals<pcl::PointNormal>(input_normals[i], 1, 0.1, normals_id);
	}
	
	PCL_INFO("Point cloud visualized, size: %d\n", size);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

bool CloudHandler::load_mesh(pcl::PolygonMesh::Ptr& input_mesh, std::string file_name)
{	
	PCL_INFO("Loading mesh %s \n", file_name.c_str());
	if (pcl::io::loadPolygonFileVTK(resource_path_ + "/" + file_name, *input_mesh) == -1) {
		PCL_ERROR("Couldn't read file: %s \n", file_name);
		return false;
	}
	PCL_INFO("Loaded mesh from: %s \n",  file_name.c_str());
}

void CloudHandler::show_mesh(pcl::PolygonMesh::Ptr& input_mesh)
{	
	pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
	viewer.addPolygonMesh(*input_mesh, "mesh");

	PCL_INFO("Mesh visualized \n");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}
