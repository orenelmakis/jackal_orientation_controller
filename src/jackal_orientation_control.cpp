#include "jackal_orientation_control/jackal_orientation_control.hpp"

orientationControl::orientationControl(ros::NodeHandle& nh): nh_(nh)
{
    initializeParameters();


}


void orientationControl::initializeParameters()
{
    nh_.getParam("wheelSize",wheelSize_);
    nh_.getParam("convergeRadius",convergeRadius_);
    nh_.getParam("convergeOrientation",convergeOrientation_);
    nh_.getParam("controllerGain",controllerGain_);

}


void orientationControl::odomSet(const Eigen::Vector3d& odom)
{
    odom_ << odom(0), odom(1), odom(2);
}


void orientationControl::refSet(const Eigen::Vector3d& refCommand)
{
    ref_ << refCommand(0), refCommand(1), refCommand(2);
}




void orientationControl::orientationControlCalculate(const Eigen::Vector3d& odom,const Eigen::Vector3d& refCommand)
{
    int directionSign = 1;
    odomSet(odom);
    refSet(refCommand);
    flagSet(odom, refCommand);
    control = Eigen::Vector2d::Zero();
    if (flag_ == 0)
    {
        
        control(1) = controllerGain_*orientationError_;
        ROS_INFO_STREAM("orientationError: " << orientationError_ << " control: " << control);
    }
    else if (flag_ == 1)
    {

        control(0) = controllerGain_*bearingError_;
        ROS_INFO_STREAM("bearingError: " << bearingError_ << " control: " << control);
    }
    else
    {
        control = Eigen::Vector2d::Zero();
    }
    if (flag_ == 0 || flag_ == 1)
    {
        status = 1;
    }
    else
    {
        status = 3;
    }
    

}

void orientationControl::flagSet(const Eigen::Vector3d& odom,const Eigen::Vector3d& refCommand)
{
    flag_ = 0;
    unitDirection_ << cos(odom(2)), sin(odom(2)), 0;
    ROS_INFO_STREAM("vectorError: " << unitDirection_);
    vectorError_ = refCommand.head(3)-odom.head(3);
    ROS_INFO_STREAM("vectorError: " << vectorError_);
    bearingError_ = unitDirection_.dot(vectorError_);
    
    orientationError_ = refCommand.tail(1)(0) - odom.tail(1)(0);
    orientationError(orientationError_);
    if(std::abs(orientationError_) < convergeOrientation_) 
    {
        if(std::abs(bearingError_) < convergeRadius_)
        {
            flag_ = 2;
        }
        else
        {
            flag_ = 1;
        }
    }


}

void orientationControl::orientationError(double& orientationError_)
{
    if(std::abs(orientationError_) > M_PI)
    {
        if(orientationError_ > 0.0)
        {
            orientationError_ = orientationError_ - 2.0 * M_PI;
        }
        else
        {
            orientationError_ = orientationError_ + 2.0 * M_PI;
        }
    }


}

