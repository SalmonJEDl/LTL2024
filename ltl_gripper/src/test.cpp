#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "testnode");
    ros::NodeHandle n;
    ros::Publisher test_pub = n.advertise<std_msgs::String>("test_pub", 100);
    ros::Rate loop_rate(10);

    while (ros::ok()){
        std_msgs::String msg;
    }
    return 0;
}
    