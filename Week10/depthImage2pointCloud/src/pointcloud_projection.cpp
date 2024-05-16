#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

using namespace std;
using namespace ros;

class PointCloudProjection{
    public:
    PointCloudProjection();
    ~PointCloudProjection();

    void DepthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void CameraInfoCllback(const sensor_msgs::CameraInfo& msg);

    pcl::PointCloud<pcl::PointXYZ> GetPointCloud(cv::Mat depth_pic);

    private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_depth_image, sub_camera_info;
    ros::Publisher pub_pointcloud;

    string depth_raw_image, depth_image_info;
    bool info_in;
    double camera_factor, camera_cx, camera_cy, camera_fx, camera_fy;
};

PointCloudProjection::PointCloudProjection():nh_(""), private_nh_("~")
{
    private_nh_.getParam("depth_raw_image",     depth_raw_image);
    private_nh_.getParam("depth_image_info",    depth_image_info);
    private_nh_.getParam("camera_factor",       camera_factor);

    sub_depth_image = nh_.subscribe(depth_raw_image, 1, &PointCloudProjection::DepthImageCallback, this);
    sub_camera_info = nh_.subscribe(depth_image_info, 1, &PointCloudProjection::CameraInfoCllback, this);
    pub_pointcloud = nh_.advertise<sensor_msgs::PointCloud2>(depth_raw_image + "/projected_points", 10);

    info_in = false;

    ROS_INFO("PointCloudProjection is created");
}
PointCloudProjection::~PointCloudProjection()
{
    ROS_INFO("PointCloudProjection is distructed");
}

void PointCloudProjection::DepthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat depth_pic;
    cv_bridge::CvImagePtr depth_ptr;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZ>);

    try
    {
        depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); 
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
    }
    depth_pic = depth_ptr->image;
    *pointcloud = GetPointCloud(depth_pic);

    // viz
    sensor_msgs::PointCloud2 detected_pointcloud_m;
    pcl::toROSMsg(*pointcloud, detected_pointcloud_m);
    detected_pointcloud_m.header.frame_id = msg->header.frame_id;
    pub_pointcloud.publish(detected_pointcloud_m);

    pointcloud = nullptr;
}

void PointCloudProjection::CameraInfoCllback(const sensor_msgs::CameraInfo& msg)
{
    camera_cx = msg.K[2];
    camera_cy = msg.K[5];
    camera_fx = msg.K[0];
    camera_fy = msg.K[4];
    info_in = true;
}

pcl::PointCloud<pcl::PointXYZ> PointCloudProjection::GetPointCloud(cv::Mat depth_pic){

    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int v = 0; v < depth_pic.rows; ++v)
    {
        for (int u = 0; u < depth_pic.cols; ++u)
        {
            float d = depth_pic.ptr<float>(v)[u];
            if (d == 0)
                continue;

            pcl::PointXYZ pt;

            pt.z = double(d) / camera_factor;
            pt.x = (u - camera_cx) * pt.z / camera_fx;
            pt.y = (v - camera_cy) * pt.z / camera_fy;

            cloud.points.push_back(pt);
        }
    }

    cloud.height   = 1;
    cloud.width    = cloud.points.size();
    cloud.is_dense = false;

    return cloud;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PointCloudProjection_node");
    PointCloudProjection detection_2_pointcloud;
    
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
