#include <ros/ros.h>
#include <jackal_orientation_control/jackal_orientation_control.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalStatus.h>
#include <tf/tf.h>

using namespace std;

class JackalOrientationControlNode
{
    public:
    JackalOrientationControlNode(ros::NodeHandle& nh);
    void initializeSubscribers();
    void initializePublishers();
    void odomSub(const nav_msgs::Odometry::ConstPtr& msg);
    void refSub(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void q2rpy(Eigen::Vector4d& q);

    private:
    ros::NodeHandle nh_;
    orientationControl jackalOrientationControl;

    // Subscribers
    ros::Subscriber poseSub_;
    ros::Subscriber refSub_;

    // Publishers
    ros::Publisher commandPub_;
    ros::Publisher GoalStatusPub_;

    // parameters for jackalOrientationControl
    Eigen::Vector3d fullRpy_;
    Eigen::Vector3d fullRef_;
    Eigen::Vector3d ref_;
    Eigen::Vector3d odom_;
    bool initialFlag = false;




};




