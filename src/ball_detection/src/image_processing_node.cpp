#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ros::Publisher position_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;
    cv::Mat hsv, mask;
    cv::resize(image, image, cv::Size(900, 600));

    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(5, 150, 150), cv::Scalar(15, 255, 255), mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std_msgs::Float32MultiArray ball_position;

    if (!contours.empty()) {
        std::vector<cv::Point> largest_contour = contours[0];
        double max_area = cv::contourArea(largest_contour);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > max_area) {
                max_area = area;
                largest_contour = contour;
            }
        }

        cv::Moments m = cv::moments(largest_contour);
        int x = static_cast<int>(m.m10 / m.m00);
        int y = static_cast<int>(m.m01 / m.m00);
        
        ball_position.data.push_back(x);
        ball_position.data.push_back(y);
        position_pub.publish(ball_position);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processing_node");
    ros::NodeHandle nh;

    position_pub = nh.advertise<std_msgs::Float32MultiArray>("ball_position", 10);
    ros::Subscriber image_sub = nh.subscribe("image_raw", 10, imageCallback);

    ros::spin();
    return 0;
}
