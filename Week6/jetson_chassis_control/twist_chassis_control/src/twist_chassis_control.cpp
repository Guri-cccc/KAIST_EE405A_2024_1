#include "twist_chassis_control/twist_chassis_control.h"

TwistChassisControl::TwistChassisControl(ros::NodeHandle* nh)
{
    _nh = nh;

    _set_velocity_pub = _nh->advertise<chassis_control::SetVelocity>("chassis_control/set_velocity", 1, this);
    _twist_cmd_sub = _nh->subscribe("cmd_vel", 1, &TwistChassisControl::_twist_cmd_cb, this);
}

void TwistChassisControl::_twist_cmd_cb(const geometry_msgs::Twist& msg)
{
    _twist_updated_time = ros::Time::now();
    _twist_cmd = msg;
}

bool TwistChassisControl::control()
{
    if(ros::Time::now() - _twist_updated_time > ros::Duration(1.0))
    {
        ROS_ERROR("Twist command out-of-dated");
        chassis_control::SetVelocity empty_cmd;
        _set_velocity_pub.publish(empty_cmd);    
        return false;
    }
    
    double vx = _twist_cmd.linear.x;
    double vy = _twist_cmd.linear.y;
    double wz = _twist_cmd.angular.z;

    // SetVelocity.velocity : [mm/s], Twist : [m/s]
    double speed = sqrt(pow(vx,2)+pow(vy,2)) * 1000.0;

    // SetVelocity.direction : -Vy
    double heading = R2D(atan2(vy, vx)) + 90.0;

    double angular = wz;

    _velocity_cmd.velocity = speed;
    _velocity_cmd.direction = heading;
    _velocity_cmd.angular = angular;

    _set_velocity_pub.publish(_velocity_cmd);

    return true;
}
