#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

float X = 0;
float Y = 0;
float Theta = 0;

void updateTheta(float theta) {
    if (theta > 180) {
        Theta = theta - 360;
    } else {
        Theta = theta;
    }
}

void ballPositionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 2) {
        X = msg->data[0];
        Y = msg->data[1];
        
        Theta = std::atan2(Y, X) * 180 / M_PI;
        updateTheta(Theta);

        if (X >= 0 && X <= 900 && Y >= 0 && Y <= 600) {
            ROS_INFO("Position: X=%.2f, Y=%.2f, Theta=%.2f", X, Y, Theta);
        } else {
            ROS_WARN("Robot keluar dari batas lapangan!");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "main_processing_node");
    ros::NodeHandle nh;

    ros::Subscriber position_sub = nh.subscribe("ball_position", 10, ballPositionCallback);

    ros::spin();
    return 0;
}
