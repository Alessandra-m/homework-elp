#include <ros/ros.h>
#include <sensor_msgs/Image.h> 
#include <cv_bridge/cv_bridge.h>
#include <elp_camera/MotionDetection.h> 

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

ros::Publisher image_pub;
ros::Publisher motion_pub;
ros::Publisher color_mask_pub;  
ros::Publisher filtered_image_pub;  

cv::Mat prev_gray;
bool first_frame = true;
const int MOTION_THRESHOLD = 2500;
std::string target_color = "red"; 

void applyColorFilter(const cv::Mat& input, cv::Mat& mask, const std::string& color) {
    cv::Mat hsv;
    cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);

    if (color == "red") {
        cv::Mat lower_red, upper_red;
        cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red);
        cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), upper_red);
        cv::bitwise_or(lower_red, upper_red, mask);
    } 
    else if (color == "green") {
        cv::inRange(hsv, cv::Scalar(40, 50, 50), cv::Scalar(80, 255, 255), mask);
    } 
    else if (color == "blue") {
        cv::inRange(hsv, cv::Scalar(100, 50, 50), cv::Scalar(140, 255, 255), mask);
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        image_pub.publish(msg);

        cv::Mat current_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        
        // Task 2
        cv::Mat gray;
        cv::cvtColor(current_frame, gray, cv::COLOR_BGR2GRAY);

        if (!first_frame) {
            cv::Mat diff;
            cv::absdiff(gray, prev_gray, diff);
            cv::threshold(diff, diff, 25, 255, cv::THRESH_BINARY);
            
            int changed_pixels = cv::countNonZero(diff);
            float diff_percentage = static_cast<float>(changed_pixels) / (gray.rows * gray.cols) * 100.0f;
            
            elp_camera::MotionDetection motion_msg;
            motion_msg.header = msg->header;
            motion_msg.detected = changed_pixels > MOTION_THRESHOLD;
            motion_msg.diff = diff_percentage;
            
            motion_pub.publish(motion_msg);
        }

        prev_gray = gray.clone();
        first_frame = false;

        // Task 3
        cv::Mat color_mask;
        applyColorFilter(current_frame, color_mask, target_color);
        
        auto mask_msg = cv_bridge::CvImage(msg->header, "mono8", color_mask).toImageMsg();
        color_mask_pub.publish(mask_msg);
        
        cv::Mat filtered_img;
        current_frame.copyTo(filtered_img, color_mask);
        auto filtered_msg = cv_bridge::CvImage(msg->header, "bgr8", filtered_img).toImageMsg();
        filtered_image_pub.publish(filtered_msg);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV Bridge error: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "elp_camera_processor");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("color", target_color, "red");
    ROS_INFO("Using color filter for: %s", target_color.c_str());

    image_pub = nh.advertise<sensor_msgs::Image>("/usb_cam/image_color", 10);
    motion_pub = nh.advertise<elp_camera::MotionDetection>("/usb_cam/motion_detection", 10);
    color_mask_pub = nh.advertise<sensor_msgs::Image>("/usb_cam/color_mask", 10);
    filtered_image_pub = nh.advertise<sensor_msgs::Image>("/usb_cam/filtered_image", 10);

    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 10, imageCallback);

    ros::spin();
    return 0;
}