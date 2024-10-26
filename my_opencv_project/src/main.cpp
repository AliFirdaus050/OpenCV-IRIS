#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap("/home/ali/Downloads/Video1.mp4"); 
    if (!cap.isOpened()) {
        cerr << "Error: Cannot open video." << endl;
        return -1;
    }

    Point2f robot_position(0, 0); 
    Point2f previous_position(0, 0); 

    const int field_center_x = 320; 
    const int field_center_y = 240;  
    const int detection_radius = 100; 
    const float threshold = 1.0; 

    while (true) {
        Mat frame;
        cap >> frame; 
        if (frame.empty()) {
            break;
        }

        Mat hsv_frame;
        cvtColor(frame, hsv_frame, COLOR_BGR2HSV);

        Scalar lower_red1(0, 120, 70);
        Scalar upper_red1(10, 255, 255);
        Scalar lower_red2(170, 120, 70);
        Scalar upper_red2(180, 255, 255);

        Mat mask1, mask2, mask;
        inRange(hsv_frame, lower_red1, upper_red1, mask1);
        inRange(hsv_frame, lower_red2, upper_red2, mask2);
        mask = mask1 | mask2; 

        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        morphologyEx(mask, mask, MORPH_OPEN, kernel);
        morphologyEx(mask, mask, MORPH_CLOSE, kernel);

        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            double max_area = 0;
            int max_index = -1;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = contourArea(contours[i]);
                if (area > max_area) {
                    max_area = area;
                    max_index = i;
                }
            }

            Moments m = moments(contours[max_index]);
            if (m.m00 != 0) {
                Point2f ball_center(m.m10 / m.m00, m.m01 / m.m00);

                if (abs(ball_center.x - field_center_x) < detection_radius &&
                    abs(ball_center.y - field_center_y) < detection_radius) {
                    
                    if (ball_center.x > field_center_x) {
                        Point2f new_position((ball_center.x - field_center_x) / 10.0, (field_center_y - ball_center.y) / 10.0);
                       
                        float distance = norm(new_position - previous_position);
                        if (distance > threshold) {
                            robot_position = new_position; 
                            previous_position = new_position; 
                        }

                        circle(frame, ball_center, 10, Scalar(0, 255, 0), -1);
                    }
                }
            }
        }

        putText(frame, "Robot Position: (" + to_string(robot_position.x) + ", " + to_string(robot_position.y) + ")",
                Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
        
        imshow("Deteksi Bola dan Posisi Robot", frame);

        if (waitKey(30) == 'q') {
            break;
        }
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
