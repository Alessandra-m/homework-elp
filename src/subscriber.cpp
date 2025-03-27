#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h> 
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>

#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>

ros::Publisher image_pub;
ros::Publisher motion_pub;

cv::Mat prev_gray;
bool first_frame = true;
const int MOTION_THRESHOLD = 5000; 

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    auto image = cv_bridge::toCvCopy(msg, "rgb8");
    // image_pub.publish(image->toImageMsg());
    image_pub.publish(msg);

    cv::Mat current_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat gray;
    cv::cvtColor(current_frame, gray, cv::COLOR_BGR2GRAY);

    if (!first_frame) {
        cv::Mat diff;
        cv::absdiff(gray, prev_gray, diff);
        cv::threshold(diff, diff, 25, 255, cv::THRESH_BINARY);
        
        int changed_pixels = cv::countNonZero(diff);
        
        std_msgs::Bool motion_msg;
        motion_msg.data = changed_pixels > MOTION_THRESHOLD;
        motion_pub.publish(motion_msg);
    }

    prev_gray = gray.clone();
    first_frame = false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "transformation_elp");
    ros::NodeHandle handle;

    ros::Subscriber subscriber = handle.subscribe("/usb_cam/image_raw", 10, imageCallback); 
    image_pub = handle.advertise<sensor_msgs::Image>("/image_color", 10);
    
    motion_pub = handle.advertise<std_msgs::Bool>("/motion_detection", 10);

    ros::spin();
    return 0;
}