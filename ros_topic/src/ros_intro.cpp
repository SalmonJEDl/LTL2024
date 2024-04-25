#include <ros/ros.h>
#include <std_msgs/Float64.h>


int main (int argc, char **argv)
{
    ros::init(argc, argv, "number_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("/x", 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("/y", 10);
    ros::Publisher pub3 = nh.advertise<std_msgs::Float64>("/z", 10);

    ros::Rate rate(1);

    while (ros::ok()) {
        std_msgs::Float64 x, y, z;
        x.data = 0.55;
        y.data = 0.0;
        z.data = 0.025;
        pub1.publish(x);
        pub2.publish(y);
        pub3.publish(z);
        rate.sleep();
    }
}