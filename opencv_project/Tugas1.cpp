#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

void loadDataset(const string& filename, vector<float>& radii, vector<float>& distances) {
    ifstream file(filename);
    string line;
    getline(file, line); 

    while (getline(file, line)) {
        stringstream ss(line);
        string radiusStr, distanceStr;
        getline(ss, radiusStr, ',');
        getline(ss, distanceStr, ',');
        radii.push_back(stof(radiusStr));
        distances.push_back(stof(distanceStr));
    }
}

int main() {
    Scalar lower(100, 150, 150);
    Scalar upper(140, 255, 255);

    VideoCapture video(0);
    if (!video.isOpened()) {
        return -1; 
    }

    Mat img, image, mask;

    vector<float> radii;
    vector<float> distances;

    loadDataset("dataset.csv", radii, distances);

    double a = 1.9832717230681362e+002; 
    double b = -5.2203446050479991e+000;
    double c = 6.3007564734525651e-002;  
    double d = -3.8840544861236067e-004;
    double e = 1.2609262049594332e-006;
    double f = -2.0486052749876437e-009;
    double g = 1.3110914781889487e-012;

    const double pixelToCm = 0.1; 

    while (true) {
        video >> img;
        if (img.empty()) break;

        cvtColor(img, image, COLOR_BGR2HSV);  
        inRange(image, lower, upper, mask);

        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            if (contourArea(contour) > 500) {
                Point2f center;
                float radius;
                minEnclosingCircle(contour, center, radius);
                
                circle(img, center, static_cast<int>(radius), Scalar(0, 0, 255), 3);
                
                cout << "Detected radius: " << radius << endl;

                double predictedDistance = a * pow(radius * pixelToCm, 6) +
                                           b * pow(radius * pixelToCm, 5) +
                                           c * pow(radius * pixelToCm, 4) +
                                           d * pow(radius * pixelToCm, 3) +
                                           e * pow(radius * pixelToCm, 2) +
                                           f * (radius * pixelToCm) +
                                           g;

                if (predictedDistance < 0) {
                    predictedDistance = 0; 
                }

                cout << "Final predicted distance: " << predictedDistance << " cm" << endl;

                string radiusText = "Radius: " + to_string(static_cast<int>(radius)) + " px";
                putText(img, radiusText, Point(center.x - 30, center.y - 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

                string distanceText = "Distance: " + to_string(static_cast<int>(predictedDistance)) + " cm";
                putText(img, distanceText, Point(center.x - 30, center.y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
            }
        }

        imshow("mask", mask);
        imshow("webcam", img);

        if (waitKey(1) == 'q') break;
    }

    video.release();
    destroyAllWindows();
    return 0;
}