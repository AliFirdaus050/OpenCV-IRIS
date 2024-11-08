#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_capture_node");
    ros::NodeHandle nh;

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("image_raw", 10);
    cv::VideoCapture cap(0);
    cv_bridge::CvImage cv_image;
    cv_image.encoding = "bgr8";

    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera");
        return -1;
    }

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame;
        if (!frame.empty()) {
            cv_image.image = frame;
            image_pub.publish(cv_image.toImageMsg());
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    cap.release();
    return 0;
}
