#include <ros/ros.h>
#include <sensor_msgs/Image.h> 
#include <cv_bridge/cv_bridge.h>
#include <elp_camera/MotionDetection.h> 

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

ros::Publisher image_pub;
ros::Publisher motion_pub;

cv::Mat prev_gray;
bool first_frame = true;
const int MOTION_THRESHOLD = 5000;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        image_pub.publish(msg);

        cv::Mat current_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
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
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV Bridge error: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_detector");
    ros::NodeHandle nh;

    image_pub = nh.advertise<sensor_msgs::Image>("/usb_cam/image_color", 10);
    motion_pub = nh.advertise<elp_camera::MotionDetection>("/usb_cam/motion_detection", 10);

    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 10, imageCallback);

    ros::spin();
    return 0;
}