#include <iostream>

#include <ros_basics_vision/ball_detector.h>

using namespace ros_tp;

int main(int argc, char** argv)
{
    cv::VideoCapture camera(std::stoi(argv[1]));
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::namedWindow("Ball Detector", cv::WINDOW_AUTOSIZE);

    cv::Mat frame;
    BallDetector bd;
    while (true) {
        camera >> frame;

        std::vector<BallDetector::Ball> balls;
        std::tie(std::ignore, std::ignore, balls) = bd.detect(frame);
        bd.annotate_image(frame, balls);

        cv::imshow("Ball Detector", frame);
        if ((cv::waitKey(1) % 256) == 27) {
            break;
        }
    }

    return 0;
}