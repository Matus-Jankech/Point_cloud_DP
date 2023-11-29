#include <cloud_handler.h>

void filter_inliers()
{	
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr main_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(main_cloud, "street_cloud.ply");
	cloud_handler.filter_outliers(main_cloud, 50, 4.0);
}

void classify_cloud()
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers_cloud, "street_cloud_inliers.ply");
	cloud_handler.DoN_based_segmentation(inliers_cloud, 0.03, 0.30);
}

void downsample_classified_clouds()
{
	CloudHandler cloud_handler;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_downsample;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class0(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class4(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(class0, "street_classified_0.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class1, "street_classified_1.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class2, "street_classified_2.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class3, "street_classified_3.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class4, "street_classified_4.ply");
	clouds_to_downsample.push_back(class0);
	clouds_to_downsample.push_back(class1);
	clouds_to_downsample.push_back(class2);
	clouds_to_downsample.push_back(class3);
	clouds_to_downsample.push_back(class4);

	cloud_handler.downsample_clouds(clouds_to_downsample);
}

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
	cloud_handler.create_mesh_Poison(street_downsampled_all, "car");
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

void separate_ground()
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers, "street_cloud_inliers.ply");
	cloud_handler.filter_ground_points(inliers);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;

	cloud_handler.load_cloud<pcl::PointXYZRGB>(ground, "street_cloud_ground.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(objects, "street_cloud_objects.ply");
	cloud_handler.set_cloud_color(ground, 90, 90, 90);

	clouds.push_back(ground);
	clouds.push_back(objects);

	cloud_handler.show_clouds(clouds, normals);
}

void test()
{	
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_data1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_data2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_data3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_handler.load_cloud<pcl::PointXYZRGB>(objects, "street_cloud_objects.ply");


	pcl::PassThrough<pcl::PointXYZRGB> pass1;
	pass1.setInputCloud(objects);
	pass1.setFilterFieldName("z");
	pass1.setFilterLimits(179, 182);

	pcl::copyPointCloud(*objects, *filtered_data1);
	pass1.setNegative(false);
	pass1.filter(*filtered_data1);

	pcl::PassThrough<pcl::PointXYZRGB> pass2;
	pass2.setInputCloud(filtered_data1);
	pass2.setFilterFieldName("x");
	pass2.setFilterLimits(0, 7);

	pcl::copyPointCloud(*objects, *filtered_data2);
	pass2.setNegative(false);
	pass2.filter(*filtered_data2);

	pcl::PassThrough<pcl::PointXYZRGB> pass3;
	pass2.setInputCloud(filtered_data2);
	pass2.setFilterFieldName("y");
	pass2.setFilterLimits(13, 16.5);

	pcl::copyPointCloud(*objects, *filtered_data3);
	pass2.setNegative(false);
	pass2.filter(*filtered_data3);

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(filtered_data3);
	sor.setLeafSize(0.07, 0.07, 0.07);
	sor.filter(*cloud_downsampled);

	cloud_handler.save_cloud<pcl::PointXYZRGB>(filtered_data3, "car.ply");
	cloud_handler.show_cloud(cloud_downsampled);
}

void car()
{
	CloudHandler cloud_handler;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_downsample;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr car(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class0(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr class2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr all(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(car, "car.ply");
	cloud_handler.DoN_based_segmentation(car, 0.03, 0.20);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(class0, "street_classified_0.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class1, "street_classified_1.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(class2, "street_classified_2.ply");
	cloud_handler.set_cloud_color(class1, 0, 0, 255);
	cloud_handler.set_cloud_color(class2, 0, 255, 0);
	clouds_to_downsample.push_back(class0);
	clouds_to_downsample.push_back(class1);
	clouds_to_downsample.push_back(class2);

	cloud_handler.downsample_clouds(clouds_to_downsample);
	cloud_handler.load_cloud<pcl::PointXYZRGB>(all, "street_downsampled_all.ply");
	cloud_handler.show_cloud(all);
}

void ground()
{
	CloudHandler cloud_handler;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_down(new pcl::PointCloud<pcl::PointXYZ>);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(ground, "street_cloud_ground.ply");

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(ground);
	sor.setLeafSize(0.20, 0.20, 0.20);
	sor.filter(*cloud_downsampled);
	cloud_handler.save_cloud<pcl::PointXYZRGB>(cloud_downsampled, "street_cloud_ground_down.ply");
	cloud_handler.load_cloud<pcl::PointXYZ>(ground_down,"street_cloud_ground_down.ply");

	cloud_handler.create_mesh_Poison(ground_down, "ground");
}

void combine_mash()
{
	CloudHandler cloud_handler;
	pcl::PolygonMesh::Ptr mesh1(new pcl::PolygonMesh);
	pcl::PolygonMesh::Ptr mesh2(new pcl::PolygonMesh);
	pcl::PolygonMesh::Ptr comb(new pcl::PolygonMesh);

	cloud_handler.load_mesh(mesh1, "ground.vtk");
	cloud_handler.load_mesh(mesh2, "car.vtk");
	*comb = *mesh1 + *mesh2;

	cloud_handler.show_mesh(comb);
}

int main(int argc, char** argv)
{	
	//downsample_classified_clouds();
	//visualize_downsampled_cloud();
	/*visualize_mesh("mesh_all");
	visualize_mesh("mesh3");
	visualize_mesh("mesh2");*/
	//visualize_inliers_outliers();

	//separate_ground();
	//test();

	
	visualize_cloud("test1.ply");
	visualize_mesh("mesh_MC1");
	visualize_mesh("mesh_Poisson1");
	visualize_cloud("test2.ply");
	visualize_mesh("mesh_Poisson2");
	car();
	//create_mesh();
	visualize_mesh("car");

	//ground();
	combine_mash();

	return (0);
}
