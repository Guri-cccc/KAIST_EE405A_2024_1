#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <detection_msgs/BoundingBoxes.h>
#include <detection_msgs/BoundingBox.h>

using namespace std;
using namespace ros;

class TargetDepth{
    public:
    TargetDepth();
    ~TargetDepth();

    private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_detection, sub_depth_iamge;
    ros::Publisher pub_target_depth;

    void DetectionCallback(const detection_msgs::BoundingBoxes& msg);
    void DepthImageCallback(const sensor_msgs::ImageConstPtr& msg);

    detection_msgs::BoundingBox GetTarget(detection_msgs::BoundingBoxes detections_);
    cv::Mat GetMaskedDepthImage(cv::Mat image_, detection_msgs::BoundingBox detection_);


    string detection_topic, depth_image_topic;
    double prev_detection_image_in;

    detection_msgs::BoundingBox target_bbox;
};

TargetDepth::TargetDepth():nh_(""), private_nh_("~")
{
    private_nh_.getParam("detection_topic",         detection_topic);
    private_nh_.getParam("depth_image_topic",       depth_image_topic);

    sub_detection = nh_.subscribe(detection_topic, 1, &TargetDepth::DetectionCallback, this);
    sub_depth_iamge = nh_.subscribe(depth_image_topic, 1, &TargetDepth::DepthImageCallback, this);
    
    pub_target_depth = nh_.advertise<sensor_msgs::Image>("/target_object", 10);

    prev_detection_image_in = 0.0;

    ROS_INFO("TargetDepth is created");
}
TargetDepth::~TargetDepth()
{
    ROS_INFO("TargetDepth is distructed");
}

void TargetDepth::DetectionCallback(const detection_msgs::BoundingBoxes& msg)
{
    if (msg.bounding_boxes.size() == 0) return;
    prev_detection_image_in = ros::Time::now().toSec();

    target_bbox = GetTarget(msg);
}

void TargetDepth::DepthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (ros::Time::now().toSec() - prev_detection_image_in > 1.0)  return;

    cv::Mat depth_pic, masked_pic;
    cv_bridge::CvImagePtr depth_ptr;

    try
    {
        depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); 
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
        return;
    }
    depth_pic = depth_ptr->image;
    masked_pic = GetMaskedDepthImage(depth_pic, target_bbox);
    
    sensor_msgs::ImagePtr target_depth_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, masked_pic).toImageMsg();
    target_depth_image->header.frame_id = "camera_depth_optical_frame";
    pub_target_depth.publish(target_depth_image);
}

// Select target object among detection result
detection_msgs::BoundingBox TargetDepth::GetTarget(detection_msgs::BoundingBoxes detections_)
{
    return detections_.bounding_boxes[0];
}

// Get target objct's detph image
cv::Mat TargetDepth::GetMaskedDepthImage(cv::Mat image_, detection_msgs::BoundingBox detection_)
{
    if (detection_.xmin >= detection_.xmax || detection_.ymin >= detection_.ymax) {
        std::cerr << "Error: Invalid bounding box coordinates." << std::endl;
        return cv::Mat();
    }

    if (detection_.xmax > image_.cols || detection_.ymax > image_.rows) {
        std::cerr << "Error: Bounding box exceeds image dimensions." << std::endl;
        return cv::Mat();
    }

    cv::Mat mask = cv::Mat::ones((detection_.ymax - detection_.ymin), (detection_.xmax - detection_.xmin), CV_16U);
    cv::Mat full_mask = cv::Mat::zeros(image_.rows, image_.cols, CV_16U);

    cv::Rect roi_full_mask(detection_.xmin, detection_.ymin, mask.cols, mask.rows);

    if (roi_full_mask.x < 0 || roi_full_mask.y < 0 || 
        roi_full_mask.x + roi_full_mask.width > full_mask.cols || 
        roi_full_mask.y + roi_full_mask.height > full_mask.rows) {
        std::cerr << "Error: ROI exceeds mask dimensions." << std::endl;
        return cv::Mat();
    }

    mask.copyTo(full_mask(roi_full_mask));

    cv::Mat mask_new;
    full_mask.convertTo(mask_new, image_.type());

    if (mask_new.size() != image_.size()) {
        std::cerr << "Error: Dimensions of mask_new and image_ do not match." << std::endl;
        return cv::Mat();
    }

    cv::Mat masked_depth_image = image_.mul(mask_new);

    return masked_depth_image;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TargetDepth_node");
    TargetDepth detection_2_pointcloud;
    
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
