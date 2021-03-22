#ifndef ROS_TP_SETUP_DETECTOR_H
#define ROS_TP_SETUP_DETECTOR_H

#include <opencv2/opencv.hpp>

namespace ros_tp {
    namespace defaults {
        struct SetupDetectorParams {
            cv::Size gaussian_kernel_sz{35, 35};
            float gaussian_sigma = 0.1;
            int num_frames_history = 30;
            double setup_area_lb_perc = 0.1;
            double setup_area_ub_perc = 0.85;
        };
    } // namespace defaults

    class SetupDetector {

        using Contour = std::vector<cv::Point>;
        using Contours = std::vector<Contour>;
        using Hierarchy = std::vector<cv::Vec4i>;
        using Ball = std::tuple<cv::Point2f, float>;

    public:
        SetupDetector();

        template <typename Params>
        SetupDetector(Params params)
        {
            _gaussian_kernel_sz = params.gaussian_kernel_sz;
            _gaussian_sigma = params.gaussian_sigma;
            _num_frames_history = params.num_frames_history;
            _history_counter = 1;
            _setup_area_lb_perc = params.setup_area_lb_perc;
            _setup_area_ub_perc = params.setup_area_ub_perc;

            _min_max_area = std::numeric_limits<double>::infinity();
        }

        void reset_counter();

        void auto_crop_to_setup(cv::Mat& img);
        void crop_to_setup(cv::Mat& img, const cv::Rect& bbox) const;

        void set_gaussian_kernel_sz(cv::Size sz);
        void set_gaussian_sigma(float sigma);
        void set_num_frames_history(int num);
        void set_setup_area_lb_perc(double perc);
        void set_setup_area_ub_perc(double perc);

        cv::Size get_gaussian_kernel_sz() const;
        float get_gaussian_sigma() const;
        int get_num_frames_history() const;
        double get_setup_area_lb_perc() const;
        double get_setup_area_ub_perc() const;

    protected:
        std::pair<cv::Rect, double> _crop_to_max_contour(cv::Mat& img);

        cv::Size _gaussian_kernel_sz;
        float _gaussian_sigma;
        int _num_frames_history;
        int _history_counter;

        double _setup_area_lb_perc;
        double _setup_area_ub_perc;

        double _min_max_area;
        cv::Rect _min_max_bbox;
    };

} // namespace ros_tp

#endif
