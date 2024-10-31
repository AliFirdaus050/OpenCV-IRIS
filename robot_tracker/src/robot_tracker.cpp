#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>

using namespace cv;

class PathFollower {
public:
    PathFollower() {
        turtle_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
        
        video_capture.open("/home/ali/Downloads/Video.mp4");
        if (!video_capture.isOpened()) {
            ROS_ERROR("Could not open video file");
            ros::shutdown();
        }

        timer = nh.createTimer(ros::Duration(0.033), &PathFollower::processFrame, this); 
    }

    void processFrame(const ros::TimerEvent&) {
        Mat frame;
        if (video_capture.read(frame)) {
            detectPath(frame);
        } else {
            ROS_INFO("End of video stream.");
            ros::shutdown();
        }
    }

    void detectPath(Mat& frame) {
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        Mat edges;
        Canny(gray, edges, 50, 150);

        std::vector<std::vector<Point>> contours;
        findContours(edges, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            int largestContourIndex = 0;
            double largestArea = 0;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = contourArea(contours[i]);
                if (area > largestArea) {
                    largestArea = area;
                    largestContourIndex = i;
                }
            }

            Moments m = moments(contours[largestContourIndex]);
            if (m.m00 > 0) {
                int x = static_cast<int>(m.m10 / m.m00);
                int y = static_cast<int>(m.m01 / m.m00);

                geometry_msgs::Twist msg;
                msg.linear.x = (x - frame.cols / 2) / 100.0; 
                msg.angular.z = (frame.rows / 2 - y) / 100.0; 
                turtle_pub.publish(msg);
            }
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher turtle_pub;
    cv::VideoCapture video_capture; 
    ros::Timer timer; 
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower");
    PathFollower follower;
    ros::spin();
    return 0;
}  