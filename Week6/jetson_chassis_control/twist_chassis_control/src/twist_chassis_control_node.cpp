#include "twist_chassis_control/twist_chassis_control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_chassis_control");
    ros::NodeHandle nh;

    ros::Rate rate(30);

    TwistChassisControl chassis_control(&nh);

    while (ros::ok())
    {
        chassis_control.control();
        ros::spinOnce();
        rate.sleep();
    }
    
}

