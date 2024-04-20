#include <cloud_handler.h>

void visualize_inliers_outliers(std::string base_path)
{	
	CloudHandler cloud_handler;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers_cloud, base_path + "/street_cloud_inliers.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(outliers_cloud, base_path + "/street_cloud_outliers.ply");
	cloud_handler.set_cloud_color(outliers_cloud, 255, 255, 255);
	clouds.push_back(inliers_cloud);
	clouds.push_back(outliers_cloud);

	cloud_handler.show_clouds(clouds);
}

void visualize_ground_objects(std::string base_path)
{
	CloudHandler cloud_handler;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(ground_cloud, base_path + "/street_cloud_ground.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(objects_cloud, base_path + "/street_cloud_objects.ply");
	cloud_handler.set_cloud_color(ground_cloud, 255, 255, 255);
	clouds.push_back(ground_cloud);
	clouds.push_back(objects_cloud);

	cloud_handler.show_clouds(clouds);
}

void visualize_classified_cloud(std::string base_path)
{	
	CloudHandler cloud_handler;
	cloud_handler.set_base_path(base_path);

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;

	int max_object_index = cloud_handler.find_max_object_index();
	max_object_index = max_object_index + 1;
	std::vector<std::array<int, 3>> colors = { {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {0, 255, 255} };

	for (int i = 0; i < max_object_index; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGB>);
			std::string file_name = "Objects/object_" + std::to_string(i) + "_c" + std::to_string(j) + ".ply";
			std::array<int, 3> color = colors[j % colors.size()];

			if (cloud_handler.load_cloud<pcl::PointXYZRGB>(object, file_name)) {
				cloud_handler.set_cloud_color(object, color[0], color[1], color[2]);
				clouds.push_back(object);
			}
		}
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_handler.load_cloud<pcl::PointXYZRGB>(ground, "/street_cloud_ground.ply");
	cloud_handler.set_cloud_color(ground, 80, 80, 80);
	clouds.push_back(ground);

	cloud_handler.show_clouds(clouds, normals);
}

void visualize_don_cloud(std::string base_path)
{
	CloudHandler cloud_handler;
	cloud_handler.set_base_path(base_path);

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;

	int max_object_index = cloud_handler.find_max_object_index();
	max_object_index = max_object_index + 1;

	for (int i = 0; i < max_object_index; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGB>);
			std::string file_name = "Objects/object_" + std::to_string(i) + "_c" + std::to_string(j) + ".ply";

			if (cloud_handler.load_cloud<pcl::PointXYZRGB>(object, file_name)) {
				cloud_handler.set_cloud_color(object, 80, 80, 80);
				clouds.push_back(object);
			}
		}

		pcl::PointCloud<pcl::PointNormal>::Ptr normal(new pcl::PointCloud<pcl::PointNormal>);
		std::string file_name = "Objects/object_" + std::to_string(i) + "_don.pcd";
		if (cloud_handler.load_cloud<pcl::PointNormal>(normal, file_name)) {
			normals.push_back(normal);
		}
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_handler.load_cloud<pcl::PointXYZRGB>(ground, "/street_cloud_ground.ply");
	cloud_handler.set_cloud_color(ground, 80, 80, 80);
	clouds.push_back(ground);

	cloud_handler.show_clouds(clouds, normals);
}

void visualize_downsampled_cloud(std::string base_path)
{
	CloudHandler cloud_handler;
	cloud_handler.set_base_path(base_path);

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;

	int max_object_index = cloud_handler.find_max_object_index();
	max_object_index = max_object_index + 1;

	for (int i = 0; i < max_object_index; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::string file_name = "Objects/object_" + std::to_string(i) + "_downsampled.ply";
		std::array<int, 3> color = cloud_handler.segment_colors_[i % cloud_handler.segment_colors_.size()];

		if (cloud_handler.load_cloud<pcl::PointXYZRGB>(object, file_name)) {
			cloud_handler.set_cloud_color(object, color[0], color[1], color[2]);
			clouds.push_back(object);
		}
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_handler.load_cloud<pcl::PointXYZRGB>(ground, "/street_cloud_ground.ply");
	cloud_handler.set_cloud_color(ground, 80, 80, 80);
	//clouds.push_back(ground);

	cloud_handler.show_clouds(clouds, normals);
}

void visualize_mesh_cloud(std::string file_name)
{	
	CloudHandler cloud_handler;
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_mesh(mesh, file_name + "_mesh.vtk");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(cloud, file_name + "_objects.ply");
	cloud_handler.show_mesh_cloud(mesh,cloud);
}

void visualize_normals(std::string file_name)
{
	CloudHandler cloud_handler;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;
	pcl::PointCloud<pcl::PointNormal>::Ptr normal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(cloud, "Testing/" + file_name + ".ply");
	cloud_handler.load_cloud<pcl::PointNormal>(normal, "Testing/" + file_name + "_normals.pcd");
	clouds.push_back(cloud);
	normals.push_back(normal);

	cloud_handler.show_clouds(clouds, normals);
}

void visualize_mesh(std::string file_name)
{
	CloudHandler cloud_handler;
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

	cloud_handler.load_mesh(mesh, file_name + ".vtk");
	cloud_handler.show_mesh(mesh);
}

void visualize_textured_mesh(std::string file_name)
{
	CloudHandler cloud_handler;
	pcl::TextureMesh::Ptr mesh(new pcl::TextureMesh);

	cloud_handler.load_textured_mesh(mesh, file_name + ".obj");
	cloud_handler.show_textured_mesh(mesh);
}

void visualize_cloud(std::string file_name)
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(cloud, file_name);
	cloud_handler.show_cloud(cloud);
}

void findObjectTest(std::string file_name)
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;

	cloud_handler.load_cloud<pcl::PointXYZRGB>(ground, "Street_cloud/street_cloud_ground.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(object, "Street_cloud/Objects/" + file_name + ".ply");
	cloud_handler.set_cloud_color(ground, 50, 50, 50);

	clouds.push_back(ground);
	clouds.push_back(object);
	cloud_handler.show_clouds(clouds, normals);
}

void findObjectPassTrough()
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered3(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers, "Roundabout_cloud/street_cloud_inliers.ply");

	// Create the pass filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(inliers);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-5, 5);

	pcl::copyPointCloud(*inliers, *filtered1);
	pass.setNegative(false);
	pass.filter(*filtered1);

	pcl::copyPointCloud(*filtered1, *filtered2);
	pass.setInputCloud(filtered1);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-500, 500);
	pass.filter(*filtered2);

	pcl::copyPointCloud(*filtered2, *filtered3);
	pass.setInputCloud(filtered2);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-500, 500);
	pass.filter(*filtered3);

	cloud_handler.save_cloud<pcl::PointXYZRGB>(filtered3, "Testing/test.ply");
	cloud_handler.show_cloud(filtered3);
}

void createMeshTest(std::string file_name)
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZ>::Ptr street_all(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr street_downsampled_all(new pcl::PointCloud<pcl::PointXYZ>);

	cloud_handler.load_cloud<pcl::PointXYZ>(street_all, "Testing/object_364.ply");
	cloud_handler.load_cloud<pcl::PointXYZ>(street_downsampled_all, "Testing/" + file_name + ".ply");
	//cloud_handler.create_mesh_GPT(street_downsampled_all, "Testing/" + file_name + "_mesh.vtk");
	cloud_handler.create_mesh_Poison(street_downsampled_all, street_all, "Testing/" + file_name + "_mesh.vtk", 9);
	//cloud_handler.create_mesh_MC(street_downsampled_all, "Testing/" + file_name + "_mesh.vtk");
}

void downsampleCloudTest(std::string file_name)
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_handler.load_cloud<pcl::PointXYZRGB>(cloud, "Testing/" + file_name + ".ply");
	std::string out_name = "Testing/" + file_name + "_downsampled.ply";
	cloud_handler.adaptive_downsampling(cloud, 0.03, 0.30, out_name);
}

void visualize_classified_cloud_test(std::string file_name)
{
	CloudHandler cloud_handler;
	cloud_handler.set_base_path("Testing");

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;

	std::vector<std::array<int, 3>> colors = { {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {0, 255, 255} };

	for (int j = 0; j < 4; j++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::string file_name2 = file_name + "_c" + std::to_string(j) + ".ply";
		std::array<int, 3> color = colors[j % colors.size()];

		if (cloud_handler.load_cloud<pcl::PointXYZRGB>(object, file_name2)) {
			cloud_handler.set_cloud_color(object, color[0], color[1], color[2]);
			clouds.push_back(object);
		}
	}

	cloud_handler.show_clouds(clouds, normals);
}

void visualize_trimmed_texture()
{
	CloudHandler cloud_handler;
	std::string base_path = "Roundabout_cloud";
	cloud_handler.set_base_path(base_path);

	cv::Mat image = cv::imread(cloud_handler.resource_path_ + "/Textures/9275.jpg");
	if (image.empty()) {
		std::cerr << "Error: Could not open or find the image!\n";
		return;
	}

	const double scale_height = 3840.0 / 1000.0;
	const double scale_width = 7680.0 / 1880.0;

	// Define the vertices of the polygon
	std::vector<cv::Point> polygon;
	polygon.push_back(cv::Point(0 / scale_width, 2490 / scale_height));
	polygon.push_back(cv::Point(275 / scale_width, 2515 / scale_height));
	polygon.push_back(cv::Point(360 / scale_width, 2560 / scale_height));
	polygon.push_back(cv::Point(425 / scale_width, 2630 / scale_height));
	polygon.push_back(cv::Point(555 / scale_width, 2650 / scale_height));
	polygon.push_back(cv::Point(545 / scale_width, 2695 / scale_height));
	polygon.push_back(cv::Point(485 / scale_width, 2705 / scale_height));
	polygon.push_back(cv::Point(580 / scale_width, 2810 / scale_height));
	polygon.push_back(cv::Point(850 / scale_width, 3025 / scale_height));
	polygon.push_back(cv::Point(1215 / scale_width, 3205 / scale_height));
	polygon.push_back(cv::Point(1480 / scale_width, 3275 / scale_height));
	polygon.push_back(cv::Point(1755 / scale_width, 3295 / scale_height));
	polygon.push_back(cv::Point(2105 / scale_width, 3325 / scale_height));
	polygon.push_back(cv::Point(2455 / scale_width, 3335 / scale_height));
	polygon.push_back(cv::Point(3030 / scale_width, 3480 / scale_height));
	polygon.push_back(cv::Point(3255 / scale_width, 3345 / scale_height));
	polygon.push_back(cv::Point(3422 / scale_width, 3145 / scale_height));
	polygon.push_back(cv::Point(3492 / scale_width, 3079 / scale_height));
	polygon.push_back(cv::Point(3570 / scale_width, 2933 / scale_height));
	polygon.push_back(cv::Point(3628 / scale_width, 2771 / scale_height));
	polygon.push_back(cv::Point(4056 / scale_width, 2773 / scale_height));
	polygon.push_back(cv::Point(4114 / scale_width, 2947 / scale_height));
	polygon.push_back(cv::Point(4208 / scale_width, 3131 / scale_height));
	polygon.push_back(cv::Point(4250 / scale_width, 3143 / scale_height));
	polygon.push_back(cv::Point(4400 / scale_width, 3337 / scale_height));
	polygon.push_back(cv::Point(5000 / scale_width, 3343 / scale_height));
	polygon.push_back(cv::Point(5501 / scale_width, 3321 / scale_height));
	polygon.push_back(cv::Point(5944 / scale_width, 3309 / scale_height));
	polygon.push_back(cv::Point(6418 / scale_width, 3233 / scale_height));
	polygon.push_back(cv::Point(6647 / scale_width, 3141 / scale_height));
	polygon.push_back(cv::Point(6919 / scale_width, 2993 / scale_height));
	polygon.push_back(cv::Point(7100 / scale_width, 2845 / scale_height));
	polygon.push_back(cv::Point(7225 / scale_width, 2711 / scale_height));
	polygon.push_back(cv::Point(7175 / scale_width, 2700 / scale_height));
	polygon.push_back(cv::Point(7159 / scale_width, 2651 / scale_height));
	polygon.push_back(cv::Point(7261 / scale_width, 2633 / scale_height));
	polygon.push_back(cv::Point(7277 / scale_width, 2649 / scale_height));
	polygon.push_back(cv::Point(7365 / scale_width, 2559 / scale_height));
	polygon.push_back(cv::Point(7445 / scale_width, 2517 / scale_height));
	polygon.push_back(cv::Point(7680 / scale_width, 2493 / scale_height));
	polygon.push_back(cv::Point(7680 / scale_width, 3840 / scale_height));
	polygon.push_back(cv::Point(0 / scale_width, 3840 / scale_height));

	cv::Size size(1880, 1000);
	cv::resize(image, image, size, 1.0, 1.0);

	// Draw polygon
	for (size_t i = 0; i < polygon.size(); ++i) {
		cv::line(image, polygon[i], polygon[(i + 1) % polygon.size()], cv::Scalar(0, 255, 0), 2);
	}

	// Display the image with the polygon lines
	cv::imshow("Image with Polygon Lines", image);
	cv::waitKey(0);
	cv::destroyAllWindows();
}

void calculate_all()
{
	CloudHandler cloud_handler;
	std::string base_path = "Roundabout_cloud";
	cloud_handler.set_base_path(base_path);

	//// INLIERS + OUTLIERS
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr street_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(street_cloud, "street_cloud.ply");
	//cloud_handler.filter_outliers(street_cloud, 20, 2.0); // First dataset

	//// GROUND + OBJECTS
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers, "street_cloud_inliers.ply");
	//cloud_handler.filter_ground_points(inliers);
	//visualize_ground_objects(base_path);

	//// CREATE GROUND MESH
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ground_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ground_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud_handler.load_cloud<pcl::PointXYZ>(ground_xyz, "street_cloud_ground.ply");
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(ground, "street_cloud_ground.ply");
	//cloud_handler.downsample_cloud(ground, 0.20, "street_cloud_ground_downsampled.ply");
	//cloud_handler.load_cloud<pcl::PointXYZ>(ground_downsampled, "street_cloud_ground_downsampled.ply");
	//cloud_handler.create_mesh_Poison(ground_downsampled, ground_xyz, "ground_mesh.vtk", 13);
	//visualize_mesh(base_path + "/ground_mesh");

	//// SEGMENT OBJECTS + CREATE MESH
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//cloud_handler.load_cloud<pcl::PointXYZRGBA>(objects, "street_cloud_objects.ply");
	//cloud_handler.cpc_segmentation(objects, true);
	//cloud_handler.downsample_objects();
	//cloud_handler.create_mesh_objects();
	//cloud_handler.combine_mesh_ground_objects();

	// TEXTURIZE MESH
	//visualize_trimmed_texture();
	pcl::PolygonMesh::Ptr mesh_all(new pcl::PolygonMesh);
	cloud_handler.load_mesh(mesh_all, "street_cloud_mesh.vtk");
	cloud_handler.texturize_mesh(mesh_all);
	//visualize_textured_mesh(base_path + "/Textures/textured_mesh");

	//// VISUALIZE RESULTS
	//visualize_mesh(base_path + "/street_cloud_mesh");
	//visualize_mesh_cloud(base_path + "/street_cloud");
	//visualize_classified_cloud(base_path);
	//visualize_downsampled_cloud(base_path);

	// MESH CHAPTER
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr car(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr car_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr car_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud_handler.load_cloud<pcl::PointXYZ>(car_xyz, "car.ply");
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(car, "car.ply");
	////cloud_handler.adaptive_downsampling(car, 0.08, 0.40, "car_downsampled.ply");
	//cloud_handler.load_cloud<pcl::PointXYZ>(car_downsampled, "car_downsampled.ply");
	//cloud_handler.create_mesh_Poison(car_xyz, car_xyz, "car_poisson.vtk", 9);
	//cloud_handler.create_mesh_Poison(car_downsampled, car_xyz, "car_poisson_downsampled.vtk", 9);
	//cloud_handler.create_mesh_MC(car_xyz, "car_MC.vtk");
	//cloud_handler.create_mesh_MC(car_downsampled, "car_MC_downsampled.vtk");
	//cloud_handler.create_mesh_GPT(car_xyz, "car_GPT.vtk");
	//cloud_handler.create_mesh_GPT(car_downsampled, "car_GPT_downsampled.vtk");
	//visualize_cloud(base_path + "/car.ply");
	//visualize_mesh(base_path + "/car_GPT");
	//visualize_mesh(base_path + "/car_MC");
	//visualize_mesh(base_path + "/car_poisson_downsampled");
	//visualize_cloud(base_path + "/car_downsampled.ply");

	/*std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;
	pcl::PointCloud<pcl::PointNormal>::Ptr car_normal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr car_downsampled_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_handler.load_cloud<pcl::PointNormal>(car_normal, "car.pcd");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(car_downsampled_RGB, "car_downsampled.ply");
	clouds.push_back(car_downsampled_RGB);
	normals.push_back(car_normal);
	cloud_handler.show_clouds(clouds,normals);*/

	//visualize_mesh(base_path + "/car_GPT_downsampled");
	//visualize_mesh(base_path + "/car_MC_downsampled");
	//visualize_mesh(base_path + "/car_poisson_downsampled");

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers, "street_cloud_inliers.ply");
	//cloud_handler.load_cloud<pcl::PointXYZ>(inliers_xyz, "street_cloud_inliers.ply");
	////cloud_handler.adaptive_downsampling(inliers, 0.08, 0.40, "inliers_downsampled.ply");
	//cloud_handler.load_cloud<pcl::PointXYZ>(inliers_downsampled, "inliers_downsampled.ply");
	//cloud_handler.create_mesh_Poison(inliers_downsampled, inliers_xyz, "street_cloud_mesh.vtk", 9);
	//visualize_cloud(base_path + "/inliers_downsampled.ply");
	//visualize_mesh(base_path + "/street_cloud_mesh");

	//pcl::PointCloud<pcl::PointXYZ>::Ptr ground_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ground_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud_handler.load_cloud<pcl::PointXYZ>(ground_downsampled, "street_cloud_ground_downsampled.ply");
	//cloud_handler.load_cloud<pcl::PointXYZ>(ground_xyz, "street_cloud_ground.ply");
	//cloud_handler.create_mesh_Poison(ground_downsampled, ground_xyz, "ground_mesh.vtk", 13);
	//cloud_handler.create_mesh_objects();
	//cloud_handler.combine_mesh_ground_objects();
	//visualize_mesh(base_path + "/street_cloud_mesh");

	//// TESTING
	//visualize_mesh("car_test");
	//visualize_mesh("Presenation/mesh_Poisson1");
	//findObjectTest("object_64");

	//std::string name = "car";
	//downsampleCloudTest(name);
	//visualize_classified_cloud_test(name);
	//createMeshTest(name + "_downsampled");
	//visualize_mesh_cloud("Testing/" + name + "_downsampled");
	//createMeshTest(name);
	//visualize_mesh_cloud("Testing/" + name);
	//visualize_normals(name);
	//findObjectPassTrough();


	//// PRESENTATION
	//visualize_cloud(base_path + "/street_cloud_inliers.ply");
	//visualize_mesh(base_path +  "/street_cloud_mesh");
	//visualize_inliers_outliers(base_path);
	//visualize_ground_objects(base_path);
	//visualize_cloud("Street_cloud/street_cloud_objects.ply");
	//visualize_cloud(base_path + "/street_cloud_ground.ply");
	//visualize_cloud(base_path + "/street_cloud_ground_downsampled.ply");
	//visualize_don_cloud(base_path);
	//visualize_classified_cloud(base_path);
	//visualize_downsampled_cloud(base_path);

	//visualize_cloud("Testing/object_371.ply");
	//std::string name = "object_364";
	//downsampleCloudTest(name);
	//createMeshTest(name + "_downsampled");
	//visualize_mesh_cloud("Testing/" + name + "_downsampled");
	//createMeshTest(name);
	//visualize_mesh_cloud("Testing/" + name);
	//visualize_normals(name);
}

int main(int argc, char** argv)
{	
	calculate_all();
	return (0);
}
