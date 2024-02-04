#include <cloud_handler.h>

void visualize_inliers_outliers()
{	
	CloudHandler cloud_handler;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers_cloud, "Street_cloud/street_cloud_inliers.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(outliers_cloud, "Street_cloud/street_cloud_outliers.ply");
	cloud_handler.set_cloud_color(outliers_cloud, 255, 255, 255);
	clouds.push_back(inliers_cloud);
	clouds.push_back(outliers_cloud);

	cloud_handler.show_clouds(clouds);
}

void visualize_ground_objects()
{
	CloudHandler cloud_handler;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(ground_cloud, "Street_cloud/street_cloud_ground.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(objects_cloud, "Street_cloud/street_cloud_objects.ply");
	cloud_handler.set_cloud_color(ground_cloud, 255, 255, 255);
	clouds.push_back(ground_cloud);
	clouds.push_back(objects_cloud);

	cloud_handler.show_clouds(clouds);
}

void visualize_classified_cloud()
{	
	CloudHandler cloud_handler;
	cloud_handler.set_base_path("Street_cloud");

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;

	int max_object_index = cloud_handler.find_max_object_index();
	max_object_index = max_object_index + 1;
	std::vector<std::array<int, 3>> colors = { {255, 0, 0}, {0, 255, 0}, {0, 0, 255} };

	for (int i = 0; i < max_object_index; i++)
	{
		for (int j = 0; j < 3; j++)
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

	cloud_handler.show_clouds(clouds, normals);
}

void visualize_downsampled_cloud()
{
	CloudHandler cloud_handler;
	cloud_handler.set_base_path("Street_cloud");

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

	cloud_handler.show_clouds(clouds, normals);
}

void visualize_mesh_cloud(std::string file_name)
{	
	CloudHandler cloud_handler;
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_mesh(mesh, file_name + "_mesh.vtk");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(cloud, file_name + ".ply");
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

void visualize_cloud(std::string file_name)
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(cloud, file_name);
	cloud_handler.show_cloud(cloud);
}

void poissonTest()
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud_handler.load_cloud<pcl::PointXYZ>(cloud, "Presenation/car.ply");
	cloud_handler.create_mesh_Poison(cloud, "car_test.vtk");

	visualize_mesh("car_test");
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
	cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers, "Street_cloud/street_cloud_inliers.ply");

	// Create the pass filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(inliers);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(180, 190);

	pcl::copyPointCloud(*inliers, *filtered1);
	pass.setNegative(false);
	pass.filter(*filtered1);

	pcl::copyPointCloud(*filtered1, *filtered2);
	pass.setInputCloud(filtered1);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-10, -5);
	pass.filter(*filtered2);

	pcl::copyPointCloud(*filtered2, *filtered3);
	pass.setInputCloud(filtered2);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-97, -96);
	pass.filter(*filtered3);

	cloud_handler.save_cloud<pcl::PointXYZRGB>(filtered3, "Testing/lamp.ply");
	cloud_handler.show_cloud(filtered3);
}

void createMeshTest(std::string file_name)
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZ>::Ptr street_downsampled_all(new pcl::PointCloud<pcl::PointXYZ>);

	cloud_handler.load_cloud<pcl::PointXYZ>(street_downsampled_all, "Testing/" + file_name + ".ply");
	//cloud_handler.create_mesh_GPT(street_downsampled_all, "Testing/" + file_name + "_mesh.vtk");
	cloud_handler.create_mesh_Poison(street_downsampled_all, "Testing/" + file_name + "_mesh.vtk");
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

void calculate_all()
{
	CloudHandler cloud_handler;
	cloud_handler.set_base_path("Street_cloud");

	//// INLIERS + OUTLIERS
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr street_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(street_cloud, "street_cloud.ply");
	//cloud_handler.filter_outliers(street_cloud, 20, 1.0);

	//// GROUND + OBJECTS
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers, "street_cloud_inliers.ply");
	//cloud_handler.filter_ground_points(inliers);

	//// CREATE GROUND MESH
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ground_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(ground, "street_cloud_ground.ply");
	//cloud_handler.downsample_cloud(ground, 0.20, "street_cloud_ground_downsampled.ply");
	//cloud_handler.load_cloud<pcl::PointXYZ>(ground_downsampled, "street_cloud_ground_downsampled.ply");
	//cloud_handler.create_mesh_Poison(ground_downsampled, "ground_mesh.vtk");

	//// SEGMENT OBJECTS + CREATE MESH
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//cloud_handler.load_cloud<pcl::PointXYZRGBA>(objects, "street_cloud_objects.ply");
	//cloud_handler.cpc_segmentation(objects, true);
	//cloud_handler.downsample_objects();
	//cloud_handler.create_mesh_objects();
	//cloud_handler.combine_mesh_ground_objects();

	//// VISUALIZE RESULTS
	//visualize_mesh("Street_cloud/street_cloud_mesh");
	//visualize_downsampled_cloud();

	//// TESTING
	//poissonTest();
	//visualize_mesh("car_test");
	//visualize_mesh("Presenation/mesh_Poisson1");
	//findObjectTest("object_64");

	//std::string name = "lamp";
	/*downsampleCloudTest(name);
	createMeshTest(name + "_downsampled");
	visualize_mesh_cloud("Testing/" + name + "_downsampled");*/
	/*createMeshTest(name);
	visualize_mesh_cloud("Testing/" + name);
	visualize_normals(name);*/

	//visualize_cloud("Street_cloud/street_cloud_inliers.ply");
	//findObjectPassTrough();


	//// PRESENTATION
	//visualize_cloud("Street_cloud/street_cloud.ply");
	//visualize_mesh("Street_cloud/street_cloud_mesh");
	//visualize_inliers_outliers();
	//visualize_ground_objects();
	//visualize_cloud("Street_cloud/street_cloud_objects.ply");
	//visualize_cloud("Street_cloud/street_cloud_ground.ply");
	//visualize_cloud("Street_cloud/street_cloud_ground_downsampled.ply");
	//visualize_classified_cloud();
	//visualize_downsampled_cloud();

	/*visualize_cloud("Testing/object_64.ply");
	std::string name = "object_64";
	downsampleCloudTest(name);
	createMeshTest(name + "_downsampled");
	visualize_mesh_cloud("Testing/" + name + "_downsampled");
	createMeshTest(name);
	visualize_mesh_cloud("Testing/" + name);
	visualize_normals(name);*/

	//visualize_mesh("Street_cloud/street_cloud_mesh");
}

int main(int argc, char** argv)
{	
	calculate_all();
	return (0);
}
