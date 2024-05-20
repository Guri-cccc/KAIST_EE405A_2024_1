
#include <memory>
#include <ros/ros.h>

#include <pid_control.h>

int main(int argc, char **argv)
{
    // node name initialization
    ros::init(argc, argv, "pid_control");
    ros::NodeHandle nh;
    PidControl selector(nh);
    ros::spin();

    return 0;
}
