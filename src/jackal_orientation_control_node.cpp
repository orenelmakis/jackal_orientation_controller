#include <jackal_orientation_control/jackal_orientation_control_node.hpp>
using namespace std;

JackalOrientationControlNode::JackalOrientationControlNode(ros::NodeHandle& nh): nh_(nh), jackalOrientationControl(nh)
{
    initializeSubscribers();
    initializePublishers();
    
}

void JackalOrientationControlNode::initializeSubscribers()
{
    poseSub_ = nh_.subscribe("/jackal0/ground_truth", 1, &JackalOrientationControlNode::odomSub, this);
    refSub_ = nh_.subscribe("/jackal0/orientation_control", 1, &JackalOrientationControlNode::refSub, this);
}

void JackalOrientationControlNode::initializePublishers()
{
    commandPub_ = nh_.advertise<geometry_msgs::Twist>("/jackal0/jackal_velocity_controller/cmd_vel", 1);
    GoalStatusPub_ = nh_.advertise<actionlib_msgs::GoalStatus>("/jackal0/jackal_velocity_controller/status", 1);
}


void JackalOrientationControlNode::odomSub(const nav_msgs::Odometry::ConstPtr& msg )
{
    Eigen::Vector4d orientation;
    orientation << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,msg->pose.pose.orientation.w;
    q2rpy(orientation);
    fullRef_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    // ROS_INFO_STREAM(fullRef_(0)<< " " << fullRef_(1) << " " << fullRef_(2));
    if (initialFlag)
    {
        odom_<< fullRef_.head(2), fullRpy_.tail(1);
        ROS_INFO_STREAM("x: " <<odom_(0)<< " y: " << odom_(1) << " z: " << odom_(2));
        jackalOrientationControl.orientationControlCalculate(odom_, ref_);

        geometry_msgs::Twist command;
        command.linear.x = jackalOrientationControl.control(0);
        command.angular.z = jackalOrientationControl.control(1);
        commandPub_.publish(command);
    }
    actionlib_msgs::GoalStatus goalStatus;
    goalStatus.status = jackalOrientationControl.status;
    GoalStatusPub_.publish(goalStatus);

}

void JackalOrientationControlNode::refSub(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    initialFlag = true;
    Eigen::Vector4d orientationCommand;
    fullRef_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    orientationCommand << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
    q2rpy(orientationCommand);
    ref_ << fullRef_.head(2), fullRpy_.tail(1);

}

void JackalOrientationControlNode::q2rpy(Eigen::Vector4d& q)
{
    tf::Quaternion q_tf = tf::Quaternion(q(0), q(1), q(2), q(3));
    tf::Matrix3x3 m(q_tf);
    Eigen::Vector3d rpyOrientation;
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    fullRpy_ << roll,pitch,yaw;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "jackal_orientation_control_node");
    ros::NodeHandle nh;
    JackalOrientationControlNode jackalOrientationControlNode(nh);
    ros::spin();
    return 0;

};