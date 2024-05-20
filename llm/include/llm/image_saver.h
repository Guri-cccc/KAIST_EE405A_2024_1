#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <llm_msgs/CallImage.h>
#include <llm_msgs/task_plan.h>

class ImageSaver{
    public:
        ImageSaver();
        void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
        void QueryCallback(const std_msgs::String::ConstPtr& msg);
    
    private:
        ros::NodeHandle nh_;

        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;

        ros::Subscriber query_sub_;
        ros::ServiceServer service_;
       
        bool HandleImageSaved(llm_msgs::CallImage::Request &req,
                                  llm_msgs::CallImage::Response &res);

        std::string package_path;
        cv::Mat last_image_;
};