#include <cloud.h>

void filter_outliers()
{
	Cloud cloud;
	if (cloud.load_cloud("street_cloud.ply"))
	{
		cloud.filter_outlier_points();
	}
}

void combine_show_clouds(std::string name1, std::string name2)
{
	Cloud cloud1, cloud2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (cloud1.load_cloud(name1) && cloud2.load_cloud(name2))
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		//cloud1.set_cloud_color(255, 255, 255);
		cloud2.set_cloud_color(200, 200, 200);

		*cloud1.cloud_ += *cloud2.cloud_;
		cloud1.show_cloud();
	}
}

void show_cloud(std::string name)
{
	Cloud cloud;
	if (cloud.load_cloud(name))
	{
		cloud.show_cloud();
	}
}

void filter_ground()
{
	Cloud cloud;
	if (cloud.load_cloud("street_cloud_inliers.ply"))
	{
		cloud.filter_ground_points(179.0, 180.0);
	}
}

void binary_to_ascii()
{
	Cloud cloud;
	if (cloud.load_cloud("street_cloud.ply"))
	{
		pcl::PLYWriter writer;
		if (writer.write<pcl::PointXYZRGB>("street_cloud_ascii.ply", *cloud.cloud_, false) == -1)
		{
			PCL_ERROR("Couldn't save file %s \n","street_cloud_ascii.ply");
		}
		PCL_INFO("Saved file %s \n", "street_cloud_ascii.ply");
	}
}

int main(int argc, char** argv)
{
	//inary_to_ascii();
	//filter_outliers();
	//combine_show_clouds("street_cloud_inliers.ply", "street_cloud_outliers.ply");
	//show_cloud("street_cloud_inliers.ply");
	//filter_ground();
	//show_cloud("street_cloud_ground.ply");
	combine_show_clouds("street_cloud_objects.ply", "street_cloud_ground.ply");
	//show_cloud("street_cloud_objects.ply");
	return (0);
}
