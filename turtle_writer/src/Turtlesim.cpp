#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/TeleportAbsolute.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class TurtleWriter {
public:
    TurtleWriter() {
        pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
        teleport_client_ = nh_.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
        
        turtlesim::TeleportAbsolute srv;
        srv.request.x = 1.0; 
        srv.request.y = 1.0;  
        srv.request.theta = 0.0;
        teleport_client_.call(srv);
    }

    void writeName() {
        std::string name = "ALIALI";
        double spacing = 1.0;
        double speed = 1.0; 

        for (char c : name) {
            moveToLetter(c, spacing, speed);
            geometry_msgs::Twist msg;
            msg.linear.x = spacing;
            pub_.publish(msg);
            ros::Duration(1.0).sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::ServiceClient teleport_client_;

    void moveToLetter(char letter, double spacing, double speed) {
        if (letter == 'A') {
            drawA(spacing, speed);
        } else if (letter == 'L') {
            drawL(spacing, speed);
        } 
    }

    void drawA(double spacing, double speed) {
        geometry_msgs::Twist msg;
        msg.linear.x = speed;
        pub_.publish(msg);
        ros::Duration(1.0).sleep();
        msg.angular.z = M_PI / 4; 
        pub_.publish(msg);
        ros::Duration(1.0).sleep(); 
        msg.angular.z = -M_PI / 2; 
        pub_.publish(msg);
        ros::Duration(1.0).sleep(); 
        msg.angular.z = M_PI / 4; 
        pub_.publish(msg);
        ros::Duration(1.0).sleep(); 
    }

    void drawL(double spacing, double speed) {
        geometry_msgs::Twist msg;
        msg.linear.x = speed;
        pub_.publish(msg);
        ros::Duration(1.0).sleep(); 
        msg.angular.z = -M_PI / 2;
        pub_.publish(msg);
        ros::Duration(1.0).sleep(); 
        msg.angular.z = M_PI / 2; 
        pub_.publish(msg);
        ros::Duration(1.0).sleep(); 
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_writer");
    TurtleWriter turtle_writer;
    turtle_writer.writeName();
    ros::spin();
    return 0;
}
