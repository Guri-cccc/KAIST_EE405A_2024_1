// Include ros header file to make ros program>
#include <ros/ros.h>
// Include header files for <geometry_msgs::PoseStamped and Twist>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
// Include header files to subscribe Joystick data with type <sensor_msgs::Joy>
#include <sensor_msgs/Joy.h>

#define SPEED_SCALE         0.05

sensor_msgs::Joy joy_cmd;

void joy_cb(const sensor_msgs::Joy& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_position_viz");
    ros::NodeHandle nh;

    ros::Rate rate(30);

    ros::Subscriber joy_sub = nh.subscribe("", 1, &joy_cb); // Edit here

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("", 1); // Edit here
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.orientation.w = 1.0;

    while(ros::ok())
    {
        if(joy_cmd.header.stamp < ros::Time::now() - ros::Duration(1.0))
            ROS_WARN("Joystick data is outdated..!");
        else
        {
            if(joy_cmd.axes.size())
            {
                ROS_INFO_STREAM("CMD_JOY (X, Y)= (" << joy_cmd.axes.at(0) << ", " << joy_cmd.axes.at(1) << ")");
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = SPEED_SCALE * ; // Edit here
                cmd_vel.linear.y = SPEED_SCALE * ; // Edit here
                ROS_INFO_STREAM("CMD_VEL (Vx, Vy)= (" << cmd_vel.linear.x << ", " << cmd_vel.linear.y << ")");
                pose.pose.position.x += ; // Edit here
                pose.pose.position.y += ; // Edit here
                ROS_INFO_STREAM("POSE (Px, Py)= (" << pose.pose.position.x << ", " << pose.pose.position.y << ")");
            }
        }
        pose.header.stamp = ros::Time::now();
        pose_pub.publish(); // Edit here

        ros::spinOnce();
        rate.sleep();
    }
}

void joy_cb(const sensor_msgs::Joy& msg)
{
    joy_cmd = msg;
}