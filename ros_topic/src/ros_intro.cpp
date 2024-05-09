#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>


int main (int argc, char **argv)
{
    ros::init(argc, argv, "number_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Publisher pub1 = nh.advertise<std_msgs::String>("/cube1", 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::String>("/cube2", 10);
    ros::Publisher pub3 = nh.advertise<std_msgs::String>("/cube3", 10);
    ros::Publisher pub4 = nh.advertise<std_msgs::String>("/cube4", 10);
    ros::Rate rate(1);

    while (ros::ok()) {
	std_msgs::String cube1, cube2, cube3, cube4;
        cube1.data = " 0.55 0.3 0.0125 red ";
        cube2.data = " 0.55 0.0 0.0125 green ";
        cube3.data = " 0.55 -0.3 0.0125 brown ";
	cube4.data = " 0.75 0.0 0.0125 yellow ";
        pub1.publish(cube1);
        pub2.publish(cube2);
        pub3.publish(cube3);
	pub4.publish(cube4);
        rate.sleep();
    }
}
