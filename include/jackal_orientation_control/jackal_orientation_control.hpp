#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>


class orientationControl
{
    public:
    orientationControl(ros::NodeHandle& nh);
    void initializeParameters();
    void odomSet(const Eigen::Vector3d& odom);
    void refSet(const Eigen::Vector3d& ref_command);
    void orientationControlCalculate(const Eigen::Vector3d& odom,const Eigen::Vector3d& ref_command);
    void flagSet(const Eigen::Vector3d& odom,const Eigen::Vector3d& ref_command);
    void orientationError(double& orientationError_);

    Eigen::Vector2d control;
    int status = 3;


    private:
    ros::NodeHandle nh_;
    Eigen::Vector3d odom_;
    Eigen::Vector3d ref_;
    Eigen::Vector3d unitDirection_;
    Eigen::Vector3d vectorError_;
    
    int flag_ ;
    double wheelSize_;
    double convergeRadius_;
    double bearingError_;
    double orientationError_;
    double convergeOrientation_;
    double controllerGain_;

    


};