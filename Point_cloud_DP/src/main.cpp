#include <cloud.h>

void filter_outliers()
{
	Cloud cloud;
	if (cloud.load_cloud("street_cloud.ply"))
	{
		cloud.filter_outlier_points();
	}
}

void combine_clouds(std::string name1, std::string name2)
{
	Cloud cloud1, cloud2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (cloud1.load_cloud(name1) && cloud2.load_cloud(name2))
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud2.set_cloud_color(0, 255, 0);

		*cloud1.cloud_ += *cloud2.cloud_;
		cloud1.show_cloud();
	}
}

void filter_ground()
{
	Cloud cloud;
	if (cloud.load_cloud("street_cloud_inliers.ply"))
	{
		cloud.filter_ground_points();
	}
}

int main(int argc, char** argv)
{
	//filter_outliers();
	//combine_clouds("street_cloud_inliers.ply", "street_cloud_outliers.ply");
	//filter_ground();
	combine_clouds("street_cloud_objects.ply", "street_cloud_ground.ply");

	return (0);
}
