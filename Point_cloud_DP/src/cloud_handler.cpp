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

void CloudHandler::downsample_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& input_clouds)
{	
	std::vector<double> voxel_sizes = { 0.22, 0.20, 0.18, 0.17, 0.15 };
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;

	unsigned int index = 0;
	for (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud : input_clouds)
	{	
		pcl::copyPointCloud(*input_cloud, *cloud_downsampled);
		sor.setInputCloud(input_cloud);
		sor.setLeafSize(voxel_sizes[index], voxel_sizes[index], voxel_sizes[index]);
		sor.filter(*cloud_downsampled);
		*cloud_all = *cloud_all + *cloud_downsampled;

		std::string file_name = "street_downsampled_" + std::to_string(index) + ".ply";
		save_cloud<pcl::PointXYZRGB>(cloud_downsampled, file_name);
		index++;
	}

	save_cloud<pcl::PointXYZRGB>(cloud_all, "street_downsampled_all.ply");
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
	save_cloud<pcl::PointNormal>(cloud_normals, normals_file);
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

void CloudHandler::DoN_based_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, double lower_limit, double upper_limit)
{	
	std::vector<double> lower_threshold = { 0.0, 0.1, 0.2, 0.3, 0.4 };
	std::vector<double> upper_threshold = { 0.1, 0.2, 0.3, 0.4, 1.0 };

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_small(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_large(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr don_cloud(new pcl::PointCloud<pcl::PointNormal>);

	// Get DoN
	std::string large_normals_file = "normlas_" + std::to_string(upper_limit) + ".pcd";
	std::string small_normals_file = "normlas_" + std::to_string(lower_limit) + ".pcd";

	// Get normlas estimation
	if (load_cloud<pcl::PointNormal>(cloud_normals_large, large_normals_file) == false)
	{
		std::cerr << "Could not load normals estimation "<< large_normals_file << ", calculating new one..." << std::endl;
		calculate_normals_estimation(input_cloud, cloud_normals_large, upper_limit);
	}
	if (load_cloud<pcl::PointNormal>(cloud_normals_small, small_normals_file) == false)
	{
		std::cerr << "Could not load normals estimation " << small_normals_file << ", calculating new one..." << std::endl;
		calculate_normals_estimation(input_cloud, cloud_normals_small, lower_limit);
	}

	calculate_don(input_cloud, don_cloud, cloud_normals_small, cloud_normals_large);


	// Build the filter
	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*input_cloud, *doncloud_filtered);
	pcl::copyPointCloud(*input_cloud, *cloud_filtered);
	
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

void CloudHandler::create_mesh_GPT(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
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
	pcl::io::savePolygonFileVTK(resource_path_ + "/mesh.vtk", triangles);
	PCL_INFO("Mesh saved \n");
}

void CloudHandler::create_mesh_Poison(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
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
	const double distanceThreshold = 0.1; 

	pcl::PointCloud<pcl::PointXYZ>::Ptr meshPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *meshPointCloud);

	// Create KD-Tree for the input cloud
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(input_cloud);

	PCL_INFO("Calculating good vertices... \n");
	std::vector<int> verticesToRemove;

	for (size_t i = 0; i < meshPointCloud->size(); ++i) {
		// Search for the closest point in the input cloud to the current point in the mesh
		std::vector<int> nearest_indices(1);
		std::vector<float> nearest_distances(1);

		pcl::PointXYZ searchPoint = meshPointCloud->at(i);
		if (kdtree.nearestKSearch(searchPoint, 1, nearest_indices, nearest_distances) > 0) {
			if (nearest_distances[0] > distanceThreshold) {
				verticesToRemove.push_back(i);
			}
		}
	}
	

	PCL_INFO("Removing bad vertices... \n");
	std::vector<pcl::Vertices>& polygons = mesh.polygons;
	std::vector<pcl::Vertices> newPolygons;

	// Remove all vertices and their associated polygons
	std::cout << "" << std::endl;
	for (size_t i = 0; i < polygons.size(); ++i) {
		std::cout << "\r";
		std::cout << "Progress: " << i / float(polygons.size()) * 100 << "%";
		pcl::Vertices vertices = polygons[i];
		pcl::Vertices newVertices;
		std::vector<uint32_t> newIndices;

		// Check if any of the vertices to remove are present in the current polygon
		for (size_t j = 0; j < vertices.vertices.size(); ++j) {
			uint32_t vertexIndex = vertices.vertices[j];
			if (std::find(verticesToRemove.begin(), verticesToRemove.end(), vertexIndex) == verticesToRemove.end()) {
				// If the vertex is not to be removed, add it to the new indices
				newIndices.push_back(vertexIndex);
			}
		}

		// If the newIndices vector is not empty, construct new vertices and store them
		if (!newIndices.empty()) {
			for (const auto& index : newIndices) {
				newVertices.vertices.push_back(index);
			}
			newPolygons.push_back(newVertices);
		}
	}
	std::cout << "" << std::endl;

	// Update the mesh with the modified polygon list
	mesh.polygons = newPolygons;

	PCL_INFO("Saving mesh... \n");
	pcl::io::savePolygonFileVTK(resource_path_ + "/mesh2.vtk", mesh);
	PCL_INFO("Mesh saved \n");
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
