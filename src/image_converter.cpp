#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include "grid_map_msgs/GridMap.h"
#include "grid_map_ros/grid_map_ros.hpp"

#include "image_transport/image_transport.h"
#include "opencv2/imgcodecs.hpp"



void occupancyGridCallback (const nav_msgs::OccupancyGridConstPtr& occupancyGrid) {

    ROS_INFO("Occupancy grid is received : %d x %d (%f)", occupancyGrid->info.width, occupancyGrid->info.height, occupancyGrid->info.resolution);


    grid_map::GridMap map;
    bool convertResult = grid_map::GridMapRosConverter::fromOccupancyGrid(*occupancyGrid, "elevation", map);
    if(!convertResult) {
        ROS_ERROR("Fail : Converting OccupancyGrid -> GridMap ");
        return;
    }


    cv_bridge::CvImage image;
    grid_map::GridMapRosConverter::toCvImage(map, "elevation", sensor_msgs::image_encodings::MONO8, image);

    std::string path = "/image.png";
    bool writeResult = cv::imwrite(path, image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
    if(!writeResult) {
        ROS_ERROR("Fail : Writing Image %s", path.c_str());
        return;
    }


    ROS_INFO("Success : Writing Image %s", path.c_str());
    ros::shutdown();

}


int main (int argc, char** argv) {

	ros::init (argc, argv, "image_converter");
	ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid> ("/map", 1, occupancyGridCallback);

	ros::spin ();
}