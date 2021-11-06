#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include "grid_map_msgs/GridMap.h"
#include "grid_map_ros/grid_map_ros.hpp"

#include "image_transport/image_transport.h"
#include "opencv2/imgcodecs.hpp"


ros::Publisher pub;

double resolution = 0.2;

int main (int argc, char** argv) {

	ros::init (argc, argv, "grid_map_publisher");
	ros::NodeHandle nh;

	int map_seq = 0;
	pub = nh.advertise<nav_msgs::OccupancyGrid> ("/grid_map", 1);

	std::string path = "/image.png";
	cv::Mat imageMat = cv::imread(path);
	std_msgs::Header header;
	header.seq = map_seq;
	header.stamp = ros::Time::now();
	header.frame_id = "map";

	cv_bridge::CvImage image = cv_bridge::CvImage(header, "rgb8", imageMat);

	sensor_msgs::ImagePtr msg = image.toImageMsg();

	grid_map::GridMap map;
	map.setBasicLayers({"elevation"});
	grid_map::GridMapRosConverter::initializeFromImage(*msg, resolution, map);
	grid_map::GridMapRosConverter::addLayerFromImage(*msg, "elevation", map, 0, 1);
  	grid_map::GridMapRosConverter::addColorLayerFromImage(*msg, "color", map);


	nav_msgs::OccupancyGrid occupancyGrid;
	grid_map::GridMapRosConverter::toOccupancyGrid(
		map,
		"elevation",
		0,
		1,
		occupancyGrid
	);


	ros::Rate rate(1);
    while (nh.ok()) {

		occupancyGrid.header.seq = map_seq++;
        occupancyGrid.header.stamp = ros::Time::now();
        occupancyGrid.header.frame_id = "map";
		occupancyGrid.info.map_load_time = ros::Time::now();		

		pub.publish (occupancyGrid);

		rate.sleep();
	}

    return 0;
}