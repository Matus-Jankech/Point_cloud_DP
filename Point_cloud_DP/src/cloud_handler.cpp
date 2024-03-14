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
	pass.setFilterLimits(179, 182); // dataset 1
	//pass.setFilterLimits(-5, 5); // dataset 2

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
	std::vector<double> voxel_sizes = { 0.13, 0.06, 0.03, 0.0 };
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
		adaptive_downsampling(object, 0.08, 0.40, out_name);
	}
}

void CloudHandler::adaptive_downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, double lower_limit, double upper_limit, std::string file_name)
{	
	std::vector<double> lower_threshold = { 0.0, 0.06, 0.12, 0.18};
	std::vector<double> upper_threshold = { 0.06, 0.12, 0.18, 1};

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

		size_t pos = file_name.find("_downsampled.ply");
		std::string in_name = file_name.substr(0, pos);
		save_cloud<pcl::PointXYZRGB>(cloud_filtered, in_name + "_c" + std::to_string(i) +  ".ply");
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
	n.setRadiusSearch(0.5);
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
	gp3.setSearchRadius(0.35);

	// Set typical values for the parameters
	gp3.setMu(2.8);
	gp3.setMaximumNearestNeighbors(1000);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	PCL_INFO("Calculating mesh... \n");
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	PCL_INFO("Saving mesh... \n");
	pcl::io::savePolygonFileVTK(resource_path_ + "/" + file_name, triangles);
	PCL_INFO("Mesh saved \n");
}

void CloudHandler::create_mesh_MC(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, std::string file_name)
{	
	// Normal estimation*
	PCL_INFO("Calculating normal estimation... \n");
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(input_cloud);

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*input_cloud, centroid);

	n.setViewPoint(centroid[0], centroid[1], centroid[2]);
	n.setInputCloud(input_cloud);
	n.setSearchMethod(tree);
	n.setRadiusSearch(0.5);
	n.setNumberOfThreads(8);
	n.compute(*normals);
		
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*input_cloud, *normals, *cloud_with_normals);

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	PCL_INFO("Calculating mesh... \n");
	pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
	pcl::PolygonMesh polygons;
	mc.setInputCloud(cloud_with_normals);
	mc.setSearchMethod(tree2);
	mc.setGridResolution(120, 120, 120);
	mc.setIsoLevel(0.02);
	mc.setDistanceIgnore(0.01);
	mc.setPercentageExtendGrid(0.00);
	mc.reconstruct(polygons);

	PCL_INFO("Saving mesh... \n");
	pcl::io::savePolygonFileVTK(resource_path_ + "/" + file_name, polygons);
	PCL_INFO("Mesh saved \n");
}

void CloudHandler::create_mesh_Poison(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
									  pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_full, 
									  std::string file_name, int depth)
{
	// Normal estimation*
	PCL_INFO("Calculating normal estimation... \n");
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(input_cloud);

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*input_cloud, centroid);

	if (input_cloud->size() < 40000) n.setViewPoint(centroid[0], centroid[1], centroid[2]);
	else n.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	n.setInputCloud(input_cloud);
	n.setSearchSurface(input_cloud_full);
	n.setSearchMethod(tree);
	n.setRadiusSearch(0.25);
	n.setNumberOfThreads(8);
	n.compute(*normals);

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*input_cloud, *normals, *cloud_with_normals);

	//save_cloud<pcl::PointNormal>(cloud_with_normals, "/Testing/object_64_normals.pcd");

	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setInputCloud(cloud_with_normals); // Set your input point cloud
	poisson.setDepth(depth); // Set the depth level of the Octree
	poisson.setSolverDivide(50);
	poisson.setIsoDivide(50);
	poisson.setPointWeight(50);
	poisson.setThreads(8);

	// Perform Poisson surface reconstruction
	PCL_INFO("Calculating mesh... \n");
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	// Define your threshold distance
	double distanceThreshold = 0;
	if (input_cloud->size() <= 1000) distanceThreshold = 0.007;
	if (input_cloud->size() > 1000) distanceThreshold = 0.013;
	if (input_cloud->size() > 40000) distanceThreshold = 0.08;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr meshPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *meshPointCloud);

	PCL_INFO("Removing bad vertices... \n");
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

	pcl::PolygonMesh::Ptr simplified_mesh(new pcl::PolygonMesh);
	pcl::surface::SimplificationRemoveUnusedVertices sruv;
	sruv.simplify(mesh, *simplified_mesh);

	//// Apply MeshSmoothingLaplacianVTK for mesh smoothing
	pcl::PolygonMesh smoothed_mesh;
	pcl::MeshSmoothingLaplacianVTK vtk;
	vtk.setInputMesh(simplified_mesh);
	vtk.setBoundarySmoothing(true);
	vtk.setFeatureEdgeSmoothing(true);
	vtk.setFeatureAngle(10);
	vtk.setEdgeAngle(150);
	vtk.setRelaxationFactor(0.03);
	vtk.setNumIter(100);
	vtk.process(smoothed_mesh);

	PCL_INFO("\nSaving mesh... \n");
	pcl::io::savePolygonFileVTK(resource_path_ + "/" + file_name, smoothed_mesh);
	PCL_INFO("Mesh saved \n");
}

void CloudHandler::create_mesh_objects()
{
	int max_object_index = find_max_object_index();
	max_object_index = max_object_index + 1;

	for (int i = 0; i < max_object_index; i++)
	{	
		pcl::PointCloud<pcl::PointXYZ>::Ptr object_full(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
		std::string in_name_full = "Objects/object_" + std::to_string(i) + ".ply";
		std::string in_name = "Objects/object_" + std::to_string(i) + "_downsampled.ply";
		std::string out_name = "Objects/object_" + std::to_string(i) + "_mesh.vtk";

		std::cout << "\n";
		load_cloud<pcl::PointXYZ>(object_full, in_name_full);
		load_cloud<pcl::PointXYZ>(object, in_name);
		create_mesh_Poison(object, object_full, out_name, 9);
	}

	std::vector<pcl::PolygonMesh::Ptr> meshes;
	for (int i = 0; i < max_object_index; i++)
	{
		pcl::PolygonMesh::Ptr object(new pcl::PolygonMesh);
		std::string file_name = "Objects/object_" + std::to_string(i) + "_mesh.vtk";

		if (load_mesh(object, file_name)) {
			meshes.push_back(object);
		}
	}

	pcl::PolygonMesh::Ptr result_mesh(new pcl::PolygonMesh);
	for (const auto& mesh : meshes) {
		*result_mesh += *mesh;
	}

	PCL_INFO("\nSaving mesh... \n");
	pcl::io::savePolygonFileVTK(resource_path_ + "/objects_mesh.vtk", *result_mesh);
	PCL_INFO("Mesh saved \n");
}

void CloudHandler::combine_mesh_ground_objects()
{
	pcl::PolygonMesh::Ptr objects(new pcl::PolygonMesh);
	pcl::PolygonMesh::Ptr ground(new pcl::PolygonMesh);
	pcl::PolygonMesh::Ptr combined(new pcl::PolygonMesh);

	load_mesh(objects, "objects_mesh.vtk");
	load_mesh(ground, "ground_mesh.vtk");
	*combined = *objects +*ground;

	PCL_INFO("\nSaving mesh... \n");
	pcl::io::savePolygonFileVTK(resource_path_ + "/street_cloud_mesh.vtk", *combined);
	PCL_INFO("Mesh saved \n");
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

void CloudHandler::texturize_mesh(pcl::PolygonMesh::Ptr& input_mesh)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(input_mesh->cloud, *cloud);

	// Create the texturemesh object that will contain our UV-mapped mesh
	pcl::TextureMesh mesh;
	mesh.cloud = input_mesh->cloud;
	std::vector< pcl::Vertices> polygon_1;

	// push faces into the texturemesh object
	polygon_1.resize(input_mesh->polygons.size());
	for (size_t i = 0; i < input_mesh->polygons.size(); ++i)
	{
		polygon_1[i] = input_mesh->polygons[i];
	}
	mesh.tex_polygons.push_back(polygon_1);
	PCL_INFO("\tInput mesh contains %d faces and %d vertices\n", mesh.tex_polygons[0].size(), cloud->points.size());
	PCL_INFO("...Done.\n");

	// Load textures and cameras poses and intrinsics
	PCL_INFO("\nLoading textures and camera poses...\n");
	pcl::texture_mapping::CameraVector my_cams;

	const boost::filesystem::path base_dir("C:/Users/admin/Documents/Visual Studio 2022/Projects/Point_cloud_DP/Resources/Roundabout_cloud/Textures");
	std::string extension(".txt");
	int cpt_cam = 0;
	for (boost::filesystem::directory_iterator it(base_dir); it != boost::filesystem::directory_iterator(); ++it)
	{
		if (boost::filesystem::is_regular_file(it->status()) && boost::filesystem::extension(it->path()) == extension)
		{
			pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
			readCamPoseFile(it->path().string(), cam);
			cam.texture_file = boost::filesystem::basename(it->path()) + ".jpg";
			my_cams.push_back(cam);
			cpt_cam++;
		}
	}
	PCL_INFO("\tLoaded %d textures.\n", my_cams.size());
	PCL_INFO("...Done.\n");

	// Display cameras to user
	PCL_INFO("\nDisplaying cameras. Press \'q\' to continue texture mapping\n");
	showCameras(my_cams, cloud);

	// Create materials for each texture (and one extra for occluded faces)
	mesh.tex_materials.resize(my_cams.size() + 1);
	for (int i = 0; i <= my_cams.size(); ++i)
	{
		pcl::TexMaterial mesh_material;
		mesh_material.tex_Ka.r = 0.2f;
		mesh_material.tex_Ka.g = 0.2f;
		mesh_material.tex_Ka.b = 0.2f;

		mesh_material.tex_Kd.r = 0.8f;
		mesh_material.tex_Kd.g = 0.8f;
		mesh_material.tex_Kd.b = 0.8f;

		mesh_material.tex_Ks.r = 1.0f;
		mesh_material.tex_Ks.g = 1.0f;
		mesh_material.tex_Ks.b = 1.0f;

		mesh_material.tex_d = 1.0f;
		mesh_material.tex_Ns = 75.0f;
		mesh_material.tex_illum = 2;

		std::stringstream tex_name;
		tex_name << "material_" << i;
		tex_name >> mesh_material.tex_name;

		if (i < my_cams.size())
			mesh_material.tex_file = my_cams[i].texture_file;
		else
			mesh_material.tex_file = "occluded.jpg";

		mesh.tex_materials[i] = mesh_material;
	}

	// Sort faces
	PCL_INFO("\nSorting faces by cameras...\n");
	map_textures_on_mesh(mesh, my_cams);

	PCL_INFO("Sorting faces by cameras done.\n");
	for (int i = 0; i <= my_cams.size(); ++i)
	{
		PCL_INFO("\tSub mesh %d contains %d faces and %d UV coordinates.\n", i, mesh.tex_polygons[i].size(), mesh.tex_coordinates[i].size());
	}

	// compute normals for the mesh
	PCL_INFO("\nEstimating normals...\n");
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	// Concatenate XYZ and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	PCL_INFO("...Done.\n");

	pcl::toPCLPointCloud2(*cloud_with_normals, mesh.cloud);

	PCL_INFO("\nSaving mesh to textured_mesh.obj\n");

	saveOBJFile(resource_path_ + "/Textures/textured_mesh.obj", mesh, 5);
}

void CloudHandler::map_textures_on_mesh(pcl::TextureMesh& mesh, const pcl::texture_mapping::CameraVector& cameras)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);

	auto tex_polygons_all = mesh.tex_polygons[0];
	std::vector<double> face_distances(tex_polygons_all.size(), std::numeric_limits<double>::max());
	std::vector<int> face_cameras(tex_polygons_all.size(), cameras.size()); // default all faces are occluded 

	// CREATE UV MAP FOR CURRENT FACES
	pcl::PointCloud<pcl::PointXY>::Ptr projections(new pcl::PointCloud<pcl::PointXY>);
	projections->resize(tex_polygons_all.size() * 3);

	for (int current_cam = 0; current_cam < static_cast<int> (cameras.size()); ++current_cam)
	{
		// transform mesh into camera's frame
		pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*mesh_cloud, *camera_cloud, cameras[current_cam].pose.inverse());

		//pcl::visualization::PCLVisualizer visu("cameras");
		//visu.addCoordinateSystem(10.0);
		//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(camera_cloud, "z");
		//visu.addPointCloud(camera_cloud, color_handler, "cloud");
		//visu.resetCamera();
		//visu.spin();

		// CREATE OCTREE
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(1.0));
		octree->setResolution(0.05);
		octree->setTreeDepth(3);
		octree->setInputCloud(camera_cloud);
		octree->addPointsFromInputCloud();

		std::cout << "Camera: " << current_cam + 1 << std::endl;

		// DIVIDE FACES BETWEEN CAMERAS
		for (int idx_face = 0; idx_face < static_cast<int> (tex_polygons_all.size()); ++idx_face)
		{
			//project each vertice (In case of sphere it should always project)
			pcl::PointXY uv_coord1, uv_coord2, uv_coord3;

			if (isFaceProjected(
				(*camera_cloud)[tex_polygons_all[idx_face].vertices[0]],
				(*camera_cloud)[tex_polygons_all[idx_face].vertices[1]],
				(*camera_cloud)[tex_polygons_all[idx_face].vertices[2]],
				uv_coord1, uv_coord2, uv_coord3)
				&&
				isPointVisible(
				(*camera_cloud)[tex_polygons_all[idx_face].vertices[0]],
				(*camera_cloud)[tex_polygons_all[idx_face].vertices[1]],
				(*camera_cloud)[tex_polygons_all[idx_face].vertices[2]],
				octree)
				)
			{
				//keep track of closest camera to polygon 
				double current_distance = distanceFromOriginToTriangleCentroid(
					(*camera_cloud)[tex_polygons_all[idx_face].vertices[0]],
					(*camera_cloud)[tex_polygons_all[idx_face].vertices[1]],
					(*camera_cloud)[tex_polygons_all[idx_face].vertices[2]]);

				if (current_distance < face_distances[idx_face])
				{
					projections->points[idx_face * 3] = uv_coord1;
					projections->points[idx_face * 3 + 1] = uv_coord2;
					projections->points[idx_face * 3 + 2] = uv_coord3;

					face_distances[idx_face] = current_distance;
					face_cameras[idx_face] = current_cam;
				}
			}
			else
			{
				// Only rewrite coordinate if face was not seen by any camera
				if (face_distances[idx_face] == std::numeric_limits<double>::max())
				{
					pcl::PointXY uv_coord1_occ, uv_coord2_occ, uv_coord3_occ;
					uv_coord1_occ.x = -1.0; uv_coord1_occ.y = -1.0;
					uv_coord2_occ.x = -1.0; uv_coord2_occ.y = -1.0;
					uv_coord3_occ.x = -1.0; uv_coord3_occ.y = -1.0;

					projections->points[idx_face * 3] = uv_coord1_occ;
					projections->points[idx_face * 3 + 1] = uv_coord2_occ;
					projections->points[idx_face * 3 + 2] = uv_coord3_occ;
				}
			}
		}
	}

	// ASSIGN FACES TO CAMERAS OR OCCLUDED 
	for (int current_cam = 0; current_cam <= static_cast<int> (cameras.size()); ++current_cam)
	{	
		if(mesh.tex_polygons.size() <= cameras.size())
		{
			std::vector<pcl::Vertices> dummy_polygons;
			mesh.tex_polygons.push_back(dummy_polygons);
		}
		mesh.tex_polygons[current_cam].clear();

		std::vector<int> current_visibility;
		for (size_t idx_face = 0; idx_face < static_cast<int> (face_cameras.size()); ++idx_face)
		{
			if (face_cameras[idx_face] == current_cam)
			{
				current_visibility.push_back(idx_face);
				mesh.tex_polygons[current_cam].push_back(tex_polygons_all[idx_face]);
			}
		}

		if (static_cast<int> (mesh.tex_coordinates.size()) <= current_cam)
		{
			std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > dummy_container;
			mesh.tex_coordinates.push_back(dummy_container);
		}
		mesh.tex_coordinates[current_cam].resize(3 * current_visibility.size());

		for (std::size_t idx_face = 0; idx_face < current_visibility.size(); ++idx_face)
		{
			mesh.tex_coordinates[current_cam][idx_face * 3](0) = (*projections)[current_visibility[idx_face] * 3].x;
			mesh.tex_coordinates[current_cam][idx_face * 3](1) = (*projections)[current_visibility[idx_face] * 3].y;

			mesh.tex_coordinates[current_cam][idx_face * 3 + 1](0) = (*projections)[current_visibility[idx_face] * 3 + 1].x;
			mesh.tex_coordinates[current_cam][idx_face * 3 + 1](1) = (*projections)[current_visibility[idx_face] * 3 + 1].y;

			mesh.tex_coordinates[current_cam][idx_face * 3 + 2](0) = (*projections)[current_visibility[idx_face] * 3 + 2].x;
			mesh.tex_coordinates[current_cam][idx_face * 3 + 2](1) = (*projections)[current_visibility[idx_face] * 3 + 2].y;
		}
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

bool CloudHandler::load_textured_mesh(pcl::TextureMesh::Ptr& input_mesh, std::string file_name)
{
	PCL_INFO("Loading mesh %s \n", file_name.c_str());
	if (pcl::io::loadOBJFile(resource_path_ + "/" + file_name, *input_mesh) == -1) {
		PCL_ERROR("Couldn't read file: %s \n", file_name);
		return false;
	}
	PCL_INFO("Loaded mesh from: %s \n", file_name.c_str());
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

void CloudHandler::show_textured_mesh(pcl::TextureMesh::Ptr& input_mesh)
{
	pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
	viewer.addTextureMesh(*input_mesh, "mesh");

	PCL_INFO("Mesh visualized \n");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

void CloudHandler::show_mesh_cloud(pcl::PolygonMesh::Ptr& input_mesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{
	pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
	viewer.addPolygonMesh(*input_mesh, "mesh");
	viewer.addPointCloud(input_cloud, "cloud");

	PCL_INFO("Mesh visualized \n");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

/** \brief Save a textureMesh object to obj file */
int CloudHandler::saveOBJFile(const std::string& file_name, const pcl::TextureMesh& tex_mesh, unsigned precision)
{
	if (tex_mesh.cloud.data.empty())
	{
		PCL_ERROR("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
		return (-1);
	}

	// Open file
	std::ofstream fs;
	fs.precision(precision);
	fs.open(file_name.c_str());

	// Define material file
	std::string mtl_file_name = file_name.substr(0, file_name.find_last_of(".")) + ".mtl";
	// Strip path for "mtllib" command
	std::string mtl_file_name_nopath = mtl_file_name;
	mtl_file_name_nopath.erase(0, mtl_file_name.find_last_of('/') + 1);

	/* Write 3D information */
	// number of points
	int nr_points = tex_mesh.cloud.width * tex_mesh.cloud.height;
	int point_size = tex_mesh.cloud.data.size() / nr_points;

	// mesh size
	int nr_meshes = tex_mesh.tex_polygons.size();
	// number of faces for header
	int nr_faces = 0;
	for (int m = 0; m < nr_meshes; ++m)
		nr_faces += tex_mesh.tex_polygons[m].size();

	// Write the header information
	fs << "####" << std::endl;
	fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
	fs << "# Vertices: " << nr_points << std::endl;
	fs << "# Faces: " << nr_faces << std::endl;
	fs << "# Material information:" << std::endl;
	fs << "mtllib " << mtl_file_name_nopath << std::endl;
	fs << "####" << std::endl;

	// Write vertex coordinates
	fs << "# Vertices" << std::endl;
	for (int i = 0; i < nr_points; ++i)
	{
		int xyz = 0;
		// "v" just be written one
		bool v_written = false;
		for (size_t d = 0; d < tex_mesh.cloud.fields.size(); ++d)
		{
			int count = tex_mesh.cloud.fields[d].count;
			if (count == 0)
				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
			int c = 0;
			// adding vertex
			if ((tex_mesh.cloud.fields[d].datatype == 7) && (
				tex_mesh.cloud.fields[d].name == "x" ||
				tex_mesh.cloud.fields[d].name == "y" ||
				tex_mesh.cloud.fields[d].name == "z"))
			{
				if (!v_written)
				{
					// write vertices beginning with v
					fs << "v ";
					v_written = true;
				}
				float value;
				memcpy(&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof(float)], sizeof(float));
				fs << value;
				if (++xyz == 3)
					break;
				fs << " ";
			}
		}
		if (xyz != 3)
		{
			PCL_ERROR("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
			return (-2);
		}
		fs << std::endl;
	}
	fs << "# " << nr_points << " vertices" << std::endl;

	// Write vertex normals
	for (int i = 0; i < nr_points; ++i)
	{
		int xyz = 0;
		// "vn" just be written one
		bool v_written = false;
		for (size_t d = 0; d < tex_mesh.cloud.fields.size(); ++d)
		{
			int count = tex_mesh.cloud.fields[d].count;
			if (count == 0)
				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
			int c = 0;
			// adding vertex
			if ((tex_mesh.cloud.fields[d].datatype == 7) && (
				tex_mesh.cloud.fields[d].name == "normal_x" ||
				tex_mesh.cloud.fields[d].name == "normal_y" ||
				tex_mesh.cloud.fields[d].name == "normal_z"))
			{
				if (!v_written)
				{
					// write vertices beginning with vn
					fs << "vn ";
					v_written = true;
				}
				float value;
				memcpy(&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof(float)], sizeof(float));
				fs << value;
				if (++xyz == 3)
					break;
				fs << " ";
			}
		}
		if (xyz != 3)
		{
			PCL_ERROR("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
			return (-2);
		}
		fs << std::endl;
	}
	// Write vertex texture with "vt" (adding latter)

	for (int m = 0; m < nr_meshes; ++m)
	{
		if (tex_mesh.tex_coordinates.size() == 0)
			continue;

		PCL_INFO("%d vertex textures in submesh %d\n", tex_mesh.tex_coordinates[m].size(), m);
		fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m << std::endl;
		for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size(); ++i)
		{
			fs << "vt ";
			fs << tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
		}
	}

	int f_idx = 0;

	// int idx_vt =0;
	PCL_INFO("Writting faces...\n");
	for (int m = 0; m < nr_meshes; ++m)
	{
		if (m > 0)
			f_idx += tex_mesh.tex_polygons[m - 1].size();

		if (tex_mesh.tex_materials.size() != 0)
		{
			fs << "# The material will be used for mesh " << m << std::endl;
			//TODO pbl here with multi texture and unseen faces
			fs << "usemtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
			fs << "# Faces" << std::endl;
		}
		for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
		{
			// Write faces with "f"
			fs << "f";
			size_t j = 0;
			// There's one UV per vertex per face, i.e., the same vertex can have
			// different UV depending on the face.
			for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size(); ++j)
			{
				unsigned int idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
				fs << " " << idx
					<< "/" << 3 * (i + f_idx) + j + 1
					<< "/" << idx; // vertex index in obj file format starting with 1
			}
			fs << std::endl;
		}
		PCL_INFO("%d faces in mesh %d \n", tex_mesh.tex_polygons[m].size(), m);
		fs << "# " << tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
	}
	fs << "# End of File";

	// Close obj file
	PCL_INFO("Closing obj file\n");
	fs.close();

	/* Write material defination for OBJ file*/
	// Open file
	PCL_INFO("Writing material files\n");
	//dont do it if no material to write
	if (tex_mesh.tex_materials.size() == 0)
		return (0);

	std::ofstream m_fs;
	m_fs.precision(precision);
	m_fs.open(mtl_file_name.c_str());

	// default
	m_fs << "#" << std::endl;
	m_fs << "# Wavefront material file" << std::endl;
	m_fs << "#" << std::endl;
	for (int m = 0; m < nr_meshes; ++m)
	{
		m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
		m_fs << "Ka " << tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
		m_fs << "Kd " << tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
		m_fs << "Ks " << tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
		m_fs << "d " << tex_mesh.tex_materials[m].tex_d << std::endl; // defines the transparency of the material to be alpha.
		m_fs << "Ns " << tex_mesh.tex_materials[m].tex_Ns << std::endl; // defines the shininess of the material to be s.
		m_fs << "illum " << tex_mesh.tex_materials[m].tex_illum << std::endl; // denotes the illumination model used by the material.
		// illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
		// illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
		m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
		m_fs << "###" << std::endl;
	}
	m_fs.close();
	return (0);
}

std::ifstream& CloudHandler::GotoLine(std::ifstream& file, unsigned int num)
{
	file.seekg(std::ios::beg);
	for (int i = 0; i < num - 1; ++i)
	{
		file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	return (file);
}

/** \brief Helper function that reads a camera file outputed by Kinfu */
bool CloudHandler::readCamPoseFile(std::string filename, pcl::TextureMapping<pcl::PointXYZ>::Camera& cam)
{
	std::ifstream myReadFile;
	myReadFile.open(filename.c_str(), ios::in);
	if (!myReadFile.is_open())
	{
		PCL_ERROR("Error opening file %d\n", filename.c_str());
		return false;
	}
	myReadFile.seekg(ios::beg);

	char current_line[1024];
	double val;

	// go to line 7 to read rotations
	GotoLine(myReadFile, 2);

	myReadFile >> val; cam.pose(0, 0) = val;
	myReadFile >> val; cam.pose(0, 1) = val;
	myReadFile >> val; cam.pose(0, 2) = val;
	myReadFile >> val; cam.pose(0, 3) = val / 100000; //TX

	myReadFile >> val; cam.pose(1, 0) = val;
	myReadFile >> val; cam.pose(1, 1) = val;
	myReadFile >> val; cam.pose(1, 2) = val;
	myReadFile >> val; cam.pose(1, 3) = val / 100000; //TY

	myReadFile >> val; cam.pose(2, 0) = val;
	myReadFile >> val; cam.pose(2, 1) = val;
	myReadFile >> val; cam.pose(2, 2) = val;
	myReadFile >> val; cam.pose(2, 3) = val / 100000; //TZ

	cam.pose(3, 0) = 0.0;
	cam.pose(3, 1) = 0.0;
	cam.pose(3, 2) = 0.0;
	cam.pose(3, 3) = 1.0; //Scale

	cam.pose = cam.pose.inverse();

	// close file
	myReadFile.close();

	return true;
}

void CloudHandler::showCameras(pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

	// visualization object
	pcl::visualization::PCLVisualizer visu("cameras");

	// add a visual for each camera at the correct pose
	for (int i = 0; i < cams.size(); ++i)
	{
		// read current camera
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam = cams[i];
		double focal = 500;
		double height = 1000;
		double width = 1000;

		// create a 5-point visual for each camera
		pcl::PointXYZ p1, p2, p3, p4, p5;
		p1.x = 0; p1.y = 0; p1.z = 0;
		double angleX = RAD2DEG(2.0 * atan(width / (2.0 * focal)));
		double angleY = RAD2DEG(2.0 * atan(height / (2.0 * focal)));
		double dist = 0.75;
		double minX, minY, maxX, maxY;
		maxX = dist * tan(atan(width / (2.0 * focal)));
		minX = -maxX;
		maxY = dist * tan(atan(height / (2.0 * focal)));
		minY = -maxY;
		p2.x = minX; p2.y = minY; p2.z = dist;
		p3.x = maxX; p3.y = minY; p3.z = dist;
		p4.x = maxX; p4.y = maxY; p4.z = dist;
		p5.x = minX; p5.y = maxY; p5.z = dist;
		p1 = pcl::transformPoint(p1, cam.pose);
		p2 = pcl::transformPoint(p2, cam.pose);
		p3 = pcl::transformPoint(p3, cam.pose);
		p4 = pcl::transformPoint(p4, cam.pose);
		p5 = pcl::transformPoint(p5, cam.pose);
		std::stringstream ss;
		ss << "Cam #" << i + 1;
		visu.addText3D(ss.str(), p1, 0.1, 1.0, 1.0, 1.0, ss.str());

		ss.str("");
		ss << "camera_" << i << "line1";
		visu.addLine(p1, p2, ss.str());
		ss.str("");
		ss << "camera_" << i << "line2";
		visu.addLine(p1, p3, ss.str());
		ss.str("");
		ss << "camera_" << i << "line3";
		visu.addLine(p1, p4, ss.str());
		ss.str("");
		ss << "camera_" << i << "line4";
		visu.addLine(p1, p5, ss.str());
		ss.str("");
		ss << "camera_" << i << "line5";
		visu.addLine(p2, p5, ss.str());
		ss.str("");
		ss << "camera_" << i << "line6";
		visu.addLine(p5, p4, ss.str());
		ss.str("");
		ss << "camera_" << i << "line7";
		visu.addLine(p4, p3, ss.str());
		ss.str("");
		ss << "camera_" << i << "line8";
		visu.addLine(p3, p2, ss.str());
	}

	// add a coordinate system
	visu.addCoordinateSystem(1.0);

	// add the mesh's cloud (colored on Z axis)
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(cloud, "z");
	visu.addPointCloud(cloud, color_handler, "cloud");

	// reset camera
	visu.resetCamera();

	// wait for user input
	visu.spin();
}

bool CloudHandler::isFaceProjected(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3,
								   pcl::PointXY& proj1, pcl::PointXY& proj2, pcl::PointXY& proj3)
{
	return (getPointUVCoordinates(p1, proj1) && getPointUVCoordinates(p2, proj2) && getPointUVCoordinates(p3, proj3));
}

bool CloudHandler::getPointUVCoordinates(const pcl::PointXYZ& pt, pcl::PointXY& UV_coordinates)
{

	float radius = 1;
	float magnitude = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
	pcl::PointXYZ normalized_point = { pt.x / magnitude, pt.y / magnitude, pt.z / magnitude };
	pcl::PointXYZ projected_point = { normalized_point.x * radius, normalized_point.y * radius, normalized_point.z * radius };

	UV_coordinates.x = atan2(projected_point.y, projected_point.x) / (2 * M_PI) + 0.5;
	// UV_coordinates.x = 1 - UV_coordinates.x;
	UV_coordinates.y = asin(projected_point.z) / M_PI + 0.5;
	UV_coordinates.y = 1 - UV_coordinates.y;

	// point is visible!
	if (UV_coordinates.x >= 0.0 && UV_coordinates.x <= 1.0 && UV_coordinates.y >= 0.0 && UV_coordinates.y <= 1.0)
		return (true); // point was visible by the camera

	// point is NOT visible by the camera
	UV_coordinates.x = -1.0f;
	UV_coordinates.y = -1.0f;
	return (false); // point was not visible by the camera
}

void CloudHandler::getTriangleCircumcscribedCircleCentroid(const pcl::PointXY& p1, const pcl::PointXY& p2, const pcl::PointXY& p3, pcl::PointXY& circumcenter, double& radius)
{
	// compute centroid's coordinates (translate back to original coordinates)
	circumcenter.x = static_cast<float> (p1.x + p2.x + p3.x) / 3;
	circumcenter.y = static_cast<float> (p1.y + p2.y + p3.y) / 3;
	double r1 = (circumcenter.x - p1.x) * (circumcenter.x - p1.x) + (circumcenter.y - p1.y) * (circumcenter.y - p1.y);
	double r2 = (circumcenter.x - p2.x) * (circumcenter.x - p2.x) + (circumcenter.y - p2.y) * (circumcenter.y - p2.y);
	double r3 = (circumcenter.x - p3.x) * (circumcenter.x - p3.x) + (circumcenter.y - p3.y) * (circumcenter.y - p3.y);

	// radius
	radius = std::sqrt(std::max(r1, std::max(r2, r3)));
}

bool CloudHandler::checkPointInsideTriangle(const pcl::PointXY& p1, const pcl::PointXY& p2, const pcl::PointXY& p3, const pcl::PointXY& pt)
{
	// Compute vectors
	Eigen::Vector2d v0, v1, v2;
	v0(0) = p3.x - p1.x; v0(1) = p3.y - p1.y; // v0= C - A
	v1(0) = p2.x - p1.x; v1(1) = p2.y - p1.y; // v1= B - A
	v2(0) = pt.x - p1.x; v2(1) = pt.y - p1.y; // v2= P - A

	// Compute dot products
	double dot00 = v0.dot(v0); // dot00 = dot(v0, v0)
	double dot01 = v0.dot(v1); // dot01 = dot(v0, v1)
	double dot02 = v0.dot(v2); // dot02 = dot(v0, v2)
	double dot11 = v1.dot(v1); // dot11 = dot(v1, v1)
	double dot12 = v1.dot(v2); // dot12 = dot(v1, v2)

	// Compute barycentric coordinates
	double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
	double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	return ((u >= 0) && (v >= 0) && (u + v < 1));
}

bool CloudHandler::isPointOccluded(const pcl::PointXYZ& pt, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree)
{
	Eigen::Vector3f direction;
	direction(0) = pt.x;
	direction(1) = pt.y;
	direction(2) = pt.z;

	pcl::Indices indices;

	pcl::PointCloud< pcl::PointXYZ>::ConstPtr cloud(new pcl::PointCloud< pcl::PointXYZ>());
	cloud = octree->getInputCloud();

	double distance_threshold = octree->getResolution();

	// raytrace
	octree->getIntersectedVoxelIndices(direction, -direction, indices);

	int nbocc = static_cast<int> (indices.size());
	for (const auto& index : indices)
	{
		// if intersected point is on the over side of the camera
		if (pt.z * (*cloud)[index].z < 0)
		{
			nbocc--;
			continue;
		}

		if (std::fabs((*cloud)[index].z - pt.z) <= distance_threshold)
		{
			// points are very close to each-other, we do not consider the occlusion
			nbocc--;
		}
	}

	return (nbocc != 0);
}

bool CloudHandler::isPointVisible(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree)
{
	return !isPointOccluded(p1, octree) && !isPointOccluded(p2, octree) && !isPointOccluded(p3, octree);
}

double CloudHandler::distanceFromOriginToTriangleCentroid(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3) {
	// Compute centroid of the triangle
	pcl::PointXYZ centroid;
	centroid.x = (p1.x + p2.x + p3.x) / 3.0;
	centroid.y = (p1.y + p2.y + p3.y) / 3.0;
	centroid.z = (p1.z + p2.z + p3.z) / 3.0;

	// Compute distance from centroid to origin
	double distance = std::sqrt(centroid.x * centroid.x + centroid.y * centroid.y + centroid.z * centroid.z);

	return distance;
}