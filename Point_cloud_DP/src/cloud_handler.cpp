#include <cloud_handler.h>

CloudHandler::CloudHandler() :
	cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
{

}

bool CloudHandler::load_point_cloud(std::string file_name)
{
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(resource_path_ + "/" + file_name, *cloud_) == -1)
	{
		PCL_ERROR("Couldn't read file %s \n", file_name.c_str());
		return (false);
	}
	PCL_INFO("Loaded %d * %d data points from %s \n", cloud_->width, cloud_->height, file_name.c_str());
	return true;
}

void CloudHandler::keyboard_event_occurred(const pcl::visualization::KeyboardEvent& event, void* viewer)
{
	if (event.getKeySym() == "q" && event.keyDown())
	{
		// Stop visualization if q pressed
	}
}

void CloudHandler::show_cloud()
{
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud_);
	PCL_INFO("Point cloud visualized \n");

	viewer.registerKeyboardCallback(keyboard_event_occurred, &viewer);
	while (!viewer.wasStopped()){}
}

