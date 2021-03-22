#include <ros_basics_vision/setup_detector.h>
#include <ros_basics_vision/adaptive_gamma.h>

using namespace ros_tp;

struct Params : public defaults::AdaptiveGammaParams {
    float clip_histogram_perc = 5.0;
    float min_brightness = 0.35;
    size_t update_rate = 30; // in asbolute number of frames
};

int main(int argc, char** argv)
{
    cv::VideoCapture camera(std::stoi(argv[1]));
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::namedWindow("Raw feed", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Setup feed (Auto)", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Setup feed (Manual)", cv::WINDOW_AUTOSIZE);

    Params params = Params();
    AdaptiveGamma ag(params);
    cv::Mat frame;

    SetupDetector sd;

    while (true) {
        camera >> frame;
        ag.adjust_image(frame);

        cv::Mat auto_cropped_frame = frame.clone();
        sd.auto_crop_to_setup(auto_cropped_frame);

        cv::Mat cropped_frame = frame.clone();
        cv::Rect bbox = cv::Rect(256, 30, 1430, 1020);
        sd.crop_to_setup(cropped_frame, bbox);

        cv::imshow("Raw feed", frame);
        cv::imshow("Setup feed (Auto)", auto_cropped_frame);
        cv::imshow("Setup feed (Manual)", cropped_frame);
        if ((cv::waitKey(1) % 256) == 27) {
            break;
        }
    }

    return 0;
}