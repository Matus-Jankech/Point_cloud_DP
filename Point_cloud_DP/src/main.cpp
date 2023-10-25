#include <cloud_handler.h>

int main(int argc, char** argv)
{	
	CloudHandler cloud_handler;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr main_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointNormal>::Ptr don_cloud(new pcl::PointCloud<pcl::PointNormal>);

	//cloud_handler.load_cloud<pcl::PointXYZRGB>(main_cloud, "street_cloud.ply");
	//cloud_handler.filter_outliers(main_cloud, 50, 4.0);

	cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers_cloud, "street_cloud_inliers.ply");
	cloud_handler.DoN_based_segmentation(inliers_cloud);

	/*cloud_handler.load_cloud<pcl::PointNormal>(don_cloud, "don.pcd");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(filtered_cloud, "street_filtered.ply");
	cloud_handler.show_cloud(don_cloud);
	cloud_handler.show_cloud(filtered_cloud);*/


	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr class0(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr class1(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr class2(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr class3(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr class4(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(class0, "street_classified_0.ply");
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(class1, "street_classified_1.ply");
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(class2, "street_classified_2.ply");
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(class3, "street_classified_3.ply");
	//cloud_handler.load_cloud<pcl::PointXYZRGB>(class4, "street_classified_4.ply");
	//cloud_handler.set_cloud_color(class1, 0, 0, 255);
	//cloud_handler.set_cloud_color(class2, 0, 255, 0);
	//cloud_handler.set_cloud_color(class3, 0, 255, 255);
	//cloud_handler.set_cloud_color(class4, 0, 0, 0);
	//clouds.push_back(class0);
	//clouds.push_back(class1);
	//clouds.push_back(class2);
	//clouds.push_back(class3);
	//clouds.push_back(class4);
	//cloud_handler.show_clouds(clouds);
	
	return (0);
}
