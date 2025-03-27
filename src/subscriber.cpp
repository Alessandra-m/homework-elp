#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h> 
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>

ros::Publisher publisher;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  auto image = cv_bridge::toCvCopy(msg, "rgb8");
  auto img = image->toImageMsg();
  publisher.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "transformation_elp");
  std::cout << "!!!!!!" << std::endl;
  ros::NodeHandle handle;

  ros::Subscriber subscriber = handle.subscribe("/usb_cam/image_raw", 10, imageCallback); 
  publisher = handle.advertise<sensor_msgs::Image>("/camera", 10);
  ros::spin();

  return 0;
}

