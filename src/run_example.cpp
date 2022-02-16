#include <ros/ros.h>




int main(int argc,char** argv)
{
    ros::init(argc, argv, "pos_publisher");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/jackal0/orientation_control", 1000);

    return 0;
}