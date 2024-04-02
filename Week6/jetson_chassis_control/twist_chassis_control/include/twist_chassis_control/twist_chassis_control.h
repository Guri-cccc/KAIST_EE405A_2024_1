#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <chassis_control/SetVelocity.h>

#define PI      3.14159265357989
#define D2R(D)  D*PI/180.0
#define R2D(R)  R*180.0/PI

class TwistChassisControl
{
    ros::NodeHandle* _nh;
    ros::Publisher _set_velocity_pub;
    ros::Subscriber _twist_cmd_sub;

    ros::Time _twist_updated_time;
    geometry_msgs::Twist _twist_cmd;
    chassis_control::SetVelocity _velocity_cmd;

    void _twist_cmd_cb(const geometry_msgs::Twist& msg);

public:
    TwistChassisControl(ros::NodeHandle* nh);

    bool control();
};