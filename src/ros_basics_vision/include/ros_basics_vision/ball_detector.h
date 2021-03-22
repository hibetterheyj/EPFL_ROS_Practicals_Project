#ifndef ROS_TP_BALL_DETECTOR_H
#define ROS_TP_BALL_DETECTOR_H

#include <opencv2/opencv.hpp>

namespace ros_tp {
    namespace defaults {
        struct BallParams {
            cv::Scalar hsv_lb{12, 118, 182};
            cv::Scalar hsv_ub{25, 255, 255};
            cv::Size gaussian_kernel_sz{11, 11};
            float gaussian_sigma = 0.1;
            size_t erode_iters = 2;
            size_t dilate_iters = 2;

            float min_radius = 35;
            float max_radius = 70;
        };
    } // namespace defaults

    class BallDetector {
    public:
        using Contours = std::vector<std::vector<cv::Point>>;
        using Hierarchy = std::vector<cv::Vec4i>;
        using Ball = std::tuple<cv::Point2f, float>;

        BallDetector();

        template <typename Params>
        BallDetector(Params params)
        {
            _lb = params.hsv_lb;
            _ub = params.hsv_ub;
            _gaussian_kernel_sz = params.gaussian_kernel_sz;
            _gaussian_sigma = params.gaussian_sigma;
            _erode_iters = params.erode_iters;
            _dilate_iters = params.dilate_iters;
            _min_radius = params.min_radius;
            _max_radius = params.max_radius;
        }

        std::tuple<Contours, Hierarchy, std::vector<Ball>> detect(const cv::Mat& img) const;
        void annotate_image(cv::Mat& img, const std::vector<Ball> ball) const;

        void set_hsv_lower_bound(const cv::Scalar& bound);
        void set_hsv_upper_bound(const cv::Scalar& bound);
        void set_gaussian_kernel_sz(cv::Size sz);
        void set_gaussian_sigma(float sz);
        void set_erode_iters(size_t iters);
        void set_dilate_iters(size_t iters);
        void set_min_radius(float radius);
        void set_max_radius(float radius);

        cv::Scalar get_hsv_lower_bound() const;
        cv::Scalar get_hsv_upper_bound() const;
        cv::Size get_gaussian_kernel_sz() const;
        float get_gaussian_sigma() const;
        size_t get_erode_iters() const;
        size_t get_dilate_iters() const;
        float get_min_radius() const;
        float get_max_radius() const;

    protected:
        cv::Scalar _lb;
        cv::Scalar _ub;
        cv::Size _gaussian_kernel_sz;
        float _gaussian_sigma;
        size_t _erode_iters;
        size_t _dilate_iters;
        float _min_radius;
        float _max_radius;
    };

} // namespace ros_tp

#endif
