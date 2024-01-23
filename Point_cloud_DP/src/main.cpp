#include <cloud_handler.h>

void visualize_inliers_outliers()
{	
	CloudHandler cloud_handler;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers_cloud, "street_cloud_inliers.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(outliers_cloud, "street_cloud_outliers.ply");
	cloud_handler.set_cloud_color(outliers_cloud, 0, 255, 0);
	clouds.push_back(inliers_cloud);
	clouds.push_back(outliers_cloud);

	cloud_handler.show_clouds(clouds);
}

void visualize_classified_cloud()
{	
	CloudHandler cloud_handler;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_downsample;
	pcl::PointCloud<pcl::PointNormal>::Ptr don_cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class0(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class4(new pcl::PointCloud<pcl::PointXYZRGB>);

	//cloud_handler.load_cloud<pcl::PointNormal>(don_cloud, "don.pcd");
	//normals.push_back(don_cloud);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(class0, "street_classified_0.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class1, "street_classified_1.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class2, "street_classified_2.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class3, "street_classified_3.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class4, "street_classified_4.ply");
	cloud_handler.set_cloud_color(class1, 0, 0, 255);
	cloud_handler.set_cloud_color(class2, 0, 255, 0);
	cloud_handler.set_cloud_color(class3, 0, 255, 255);
	cloud_handler.set_cloud_color(class4, 255, 255, 0);
	clouds.push_back(class0);
	clouds.push_back(class1);
	clouds.push_back(class2);
	clouds.push_back(class3);
	clouds.push_back(class4);

	cloud_handler.show_clouds(clouds, normals);
}

void visualize_downsampled_cloud()
{
	CloudHandler cloud_handler;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class_downsampled_0(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class_downsampled_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class_downsampled_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class_downsampled_3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class_downsampled_4(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(class_downsampled_0, "street_downsampled_0.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class_downsampled_1, "street_downsampled_1.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class_downsampled_2, "street_downsampled_2.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class_downsampled_3, "street_downsampled_3.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class_downsampled_4, "street_downsampled_4.ply");
	/*cloud_handler.set_cloud_color(class_downsampled_1, 0, 0, 255);
	cloud_handler.set_cloud_color(class_downsampled_2, 0, 255, 0);
	cloud_handler.set_cloud_color(class_downsampled_3, 0, 255, 255);
	cloud_handler.set_cloud_color(class_downsampled_4, 255, 255, 0);*/
	clouds.push_back(class_downsampled_0);
	clouds.push_back(class_downsampled_1);
	clouds.push_back(class_downsampled_2);
	clouds.push_back(class_downsampled_3);
	clouds.push_back(class_downsampled_4);

	cloud_handler.show_clouds(clouds, normals);
}

void create_mesh()
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZ>::Ptr street_downsampled_all(new pcl::PointCloud<pcl::PointXYZ>);

	cloud_handler.load_cloud<pcl::PointXYZ>(street_downsampled_all, "street_downsampled_all.ply");
	//cloud_handler.create_mesh_GPT(street_downsampled_all);
	cloud_handler.create_mesh_Poison(street_downsampled_all, "speed");
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
	visualize_mesh("Street_cloud/street_cloud_mesh");

	//// TESTING
	//poissonTest();
	//visualize_mesh("car_test");
	//visualize_mesh("Presenation/mesh_Poisson1");
}

int main(int argc, char** argv)
{	

	calculate_all();

	return (0);
}
