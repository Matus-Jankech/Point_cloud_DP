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
	cloud_handler.create_mesh(street_downsampled_all);
}

void visualize_mesh()
{	
	CloudHandler cloud_handler;
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

	cloud_handler.load_mesh(mesh, "mesh.vtk");
	cloud_handler.show_mesh(mesh);
}

int main(int argc, char** argv)
{	
	//downsample_classified_clouds();
	visualize_downsampled_cloud();
	//create_mesh();
	visualize_mesh();

	return (0);
}
