#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/centroid.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <point_tf/point_tf.h>

using namespace std;
using namespace ros;

class PositionEstimation{
    public:
    PositionEstimation();
    ~PositionEstimation();

    private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_target_pointcloud;
    ros::Publisher pub_target_center, pub_target_pose;

    void TargetPointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    geometry_msgs::Point DynamicTF(geometry_msgs::Point point_, string from_frame_, string to_frame_);
    geometry_msgs::Point StaticTF(geometry_msgs::Point point_, geometry_msgs::Transform tf_, bool reverse_);

    string pointcloud_topic;
    bool pointcloud_in;
    double x_bias, y_bias, z_bias;
};

PositionEstimation::PositionEstimation():nh_(""), private_nh_("~")
{
    private_nh_.getParam("pointcloud_topic",            pointcloud_topic);
    private_nh_.getParam("x_bias",                      x_bias);
    private_nh_.getParam("y_bias",                      y_bias);
    private_nh_.getParam("z_bias",                      z_bias);

    sub_target_pointcloud = nh_.subscribe(pointcloud_topic, 1, &PositionEstimation::TargetPointcloudCallback, this);
    
    pub_target_center = nh_.advertise<geometry_msgs::Point>("/target_position", 10);
    pub_target_pose = nh_.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);

    pointcloud_in = false;

    ROS_INFO("PositionEstimation is created");
}
PositionEstimation::~PositionEstimation()
{
    ROS_INFO("PositionEstimation is distructed");
}

#define M_PI 3.14159265358929
#define DEG2RAD(deg) deg*M_PI/180.0
double sat(double val, double lim)
{
    if(abs(val) > lim)
        val > 0 ? val=abs(lim) : val=-abs(lim);
    return val;
}

void PositionEstimation::TargetPointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_in);

    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    for (auto p: cloud_in->points)
    {
        centroid.add (p);
    }
    pcl::PointXYZ center;
    centroid.get (center);

    geometry_msgs::Point center_point;
    
    double cam_tilt = DEG2RAD(17.5);
    center_point.x = center.x + x_bias;
    center_point.y = (center.y*sin(cam_tilt) + center.z*cos(cam_tilt))*((1.0 - sat(center.x, 0.125))/1.0) + y_bias;
    center_point.z = z_bias;
    ROS_INFO_STREAM("Px: " << center_point.x << ", Py: " << center_point.y << ", Pz:" << center_point.z);
    
    pub_target_center.publish(center_point);
    
    geometry_msgs::PoseStamped center_pose;
    center_pose.pose.position = center_point;
    center_pose.pose.orientation.w = 1.0;
    center_pose.header.frame_id = msg->header.frame_id;
    pub_target_pose.publish(center_pose);
}

geometry_msgs::Point PositionEstimation::DynamicTF(geometry_msgs::Point point_, string from_frame_, string to_frame_)
{
    geometry_msgs::Point transformed_point;
    geometry_msgs::TransformStamped pose_transform;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    try {
        pose_transform = tf_buffer.lookupTransform(to_frame_, from_frame_, ros::Time(0), ros::Duration(1.0));
        tf2::doTransform(point_, transformed_point, pose_transform);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
    }

    return transformed_point;
}

geometry_msgs::Point PositionEstimation::StaticTF(geometry_msgs::Point point_, geometry_msgs::Transform tf_, bool reverse_)
{
    return PointTF::PointTF::Transform(point_, tf_, reverse_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PositionEstimation_node");
    PositionEstimation detection_2_pointcloud;
    
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
