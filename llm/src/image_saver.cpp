#include "llm/image_saver.h"

ImageSaver::ImageSaver()
    : it_(nh_) {
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageSaver::ImageCallback, this);
    query_sub_ = nh_.subscribe("/vision_query", 1, &ImageSaver::QueryCallback, this);
    service_ = nh_.advertiseService("image_saved", &ImageSaver::HandleImageSaved, this);
    package_path = ros::package::getPath("llm");
};

void ImageSaver::ImageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        if (!cv_ptr) {
            ROS_ERROR("Failed to convert image: cv_ptr is null");
            return;
        }

        // std::cout << "Successfully converted to CvImage" << std::endl;
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
        // std::cout << "Successfully converted RGB to BGR" << std::endl;
    }
    catch (const cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    catch (const cv::Exception& e){
        ROS_ERROR("OpenCV exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.empty()) {
        ROS_ERROR("Converted image is empty");
        return;
    }
    // printf("debug1111");
    last_image_ = cv_ptr->image;
    // std::cout << last_image_ << std::endl;
}

void ImageSaver::QueryCallback(const std_msgs::String::ConstPtr& msg){
    std::cout << msg->data.c_str() << std::endl;
    if (!last_image_.empty()) {
        std::string file_path = package_path + "/images/" + "gpt_vision" + ".jpg";
        std::cout << file_path << std::endl;
        bool saved = cv::imwrite(file_path, last_image_);
        if (!saved) {
            ROS_ERROR("Failed to save image at %s", file_path.c_str());
        } else {
            ROS_INFO("Image saved successfully at %s", file_path.c_str());
        }
    } else {
        ROS_WARN("No image available to save.");
    }
}

bool ImageSaver::HandleImageSaved(llm_msgs::CallImage::Request &req,
                                  llm_msgs::CallImage::Response &res){
    if (req.saved) {
        res.success = true;
        ROS_INFO("Service image_saved: Image was successfully saved.");
    }
    else {
        res.success = false;
    }
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "image_saver_node_");
    ImageSaver image_saver;
    ros::spin();
    return 0;
}