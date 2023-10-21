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

	/*cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers_cloud, "street_cloud_inliers.ply");
	cloud_handler.DoN_based_segmentation(main_cloud);
	cloud_handler.show_cloud<pcl::PointXYZRGB>(inliers_cloud);*/

	/*cloud_handler.load_cloud<pcl::PointNormal>(don_cloud, "don.pcd");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(filtered_cloud, "street_filtered.ply");
	cloud_handler.show_cloud(don_cloud);
	cloud_handler.show_cloud(filtered_cloud);*/

	cloud_handler.load_cloud<pcl::PointXYZRGB>(inliers_cloud, "street_cloud_inliers.ply");
	cloud_handler.load_cloud<pcl::PointXYZRGB>(outliers_cloud, "street_cloud_outliers.ply");
	cloud_handler.set_cloud_color(outliers_cloud, 0, 255, 0);
	clouds.push_back(outliers_cloud);
	clouds.push_back(inliers_cloud);

	cloud_handler.show_clouds(clouds);
	
	return (0);
}
