#ifndef ROS_TP_ADAPTIVE_GAMMA_H
#define ROS_TP_ADAPTIVE_GAMMA_H

#include <opencv2/opencv.hpp>

namespace ros_tp {
    namespace defaults {
        struct AdaptiveGammaParams {
            float clip_histogram_perc = 1;
            float min_brightness = 1.0;
            int update_rate = 30; // in asbolute number of frames
        };
    } // namespace defaults

    class AdaptiveGamma {
    public:
        AdaptiveGamma();

        template <typename Params>
        AdaptiveGamma(Params params)
        {
            _clip_histogram_perc = params.clip_histogram_perc;
            _min_brightness = params.min_brightness;
            _update_rate = params.update_rate;
            _iter_counter = 0;
        }

        void adjust_image(cv::Mat& img);
        std::pair<float, float> compute_bc_gains(const cv::Mat& img); // brightness and contrast gains (alpha, beta)
        void convert(cv::Mat& img, float alpha, float beta);

        void set_clip_histogram_perc(float perc);
        void set_min_brightness(float brightness);
        void set_udpate_rate(int rate);

        float get_clip_histogram_perc() const;
        float get_min_brightness() const;
        int get_update_rate() const;

    protected:
        float _alpha;
        float _beta;
        float _clip_histogram_perc;
        float _min_brightness;
        int _update_rate;
        int _iter_counter;
    };

} // namespace ros_tp

#endif
