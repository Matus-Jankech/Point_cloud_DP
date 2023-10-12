#include <cloud.h>

void filter_outliers()
{
	Cloud cloud;

	if (cloud.load_cloud("street_cloud.ply"))
	{
		cloud.filter_outliers();
	}
}

void combine_inlier_outlier()
{
	Cloud cloud1, cloud2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (cloud1.load_cloud("street_cloud_inliers.ply") && cloud2.load_cloud("street_cloud_outliers.ply"))
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud1.set_cloud_color(255, 0, 0);
		cloud2.set_cloud_color(0, 255, 0);

		*cloud1.cloud_ += *cloud2.cloud_;
		cloud1.show_cloud();
	}
}

int main(int argc, char** argv)
{
	//filter_outliers();
	combine_inlier_outlier();
	return (0);
}
