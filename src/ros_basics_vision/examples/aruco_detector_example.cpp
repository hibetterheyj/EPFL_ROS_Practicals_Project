#include <ros_basics_vision/aruco_detector.h>

using namespace ros_tp;

int main(int argc, char** argv)
{
    cv::VideoCapture camera(std::stoi(argv[1]));
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cv::namedWindow("Aruco Detector", cv::WINDOW_AUTOSIZE);

    cv::Mat frame;
    ArucoDetector ad;
    while (true) {
        camera >> frame;

        ad.detect(frame);
        ad.annotate_image(frame);

        cv::imshow("Aruco Detector", frame);
        if ((cv::waitKey(1) % 256) == 27) {
            break;
        }
    }

    return 0;
}