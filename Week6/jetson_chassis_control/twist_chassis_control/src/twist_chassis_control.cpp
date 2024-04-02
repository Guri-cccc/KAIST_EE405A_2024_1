#include "twist_chassis_control/twist_chassis_control.h"

TwistChassisControl::TwistChassisControl(ros::NodeHandle* nh)
{
    _nh = nh;

    // TODO
    _set_velocity_pub = _nh->advertise<chassis_control::SetVelocity>("", 1, this);
    _twist_cmd_sub = _nh->subscribe("", 1, , this);
}

void TwistChassisControl::_twist_cmd_cb(const geometry_msgs::Twist& msg)
{
    // TODO
    _twist_updated_time = ;
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

    // TODO
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;

    // Hint: SetVelocity.velocity : [mm/s], Twist : [m/s]
    double speed = 0.0;

    // Hint: SetVelocity.direction 0 dgree: : -Vy
    double heading = 0.0;

    double angular = 0.0;

    _velocity_cmd.velocity = speed;
    _velocity_cmd.direction = heading;
    _velocity_cmd.angular = angular;

    _set_velocity_pub.publish(_velocity_cmd);

    return true;
}
