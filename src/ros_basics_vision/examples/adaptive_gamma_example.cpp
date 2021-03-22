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

    cv::namedWindow("Without adaptive gamma", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("With adaptive gamma", cv::WINDOW_AUTOSIZE);

    Params params = Params();
    AdaptiveGamma ag(params);
    cv::Mat frame;

    while (true) {
        camera >> frame;
        cv::Mat adapted_frame = frame.clone();

        ag.adjust_image(adapted_frame);

        cv::imshow("Without adaptive gamma", frame);
        cv::imshow("With adaptive gamma", adapted_frame);
        if ((cv::waitKey(1) % 256) == 27) {
            break;
        }
    }

    return 0;
}