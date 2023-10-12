#include <cloud_handler.h>

int main(int argc, char** argv)
{
	CloudHandler cloud_handler;

	cloud_handler.load_point_cloud("street_cloud.ply");
	cloud_handler.show_cloud();

	return (0);
}