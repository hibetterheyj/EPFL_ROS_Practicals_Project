#include <ros_basics_vision/ball_detector.h>

namespace ros_tp {
    BallDetector::BallDetector() : BallDetector(defaults::BallParams())
    {
    }

    std::tuple<BallDetector::Contours, BallDetector::Hierarchy, std::vector<BallDetector::Ball>> BallDetector::detect(const cv::Mat& img) const
    {
        cv::Mat frame = img.clone();
        cv::GaussianBlur(frame, frame, _gaussian_kernel_sz, _gaussian_sigma, _gaussian_sigma);

        cv::Mat mask, kernel;
        cv::cvtColor(frame, mask, cv::COLOR_BGR2HSV);
        cv::inRange(mask, _lb, _ub, mask);
        cv::erode(mask, mask, kernel, cv::Point(-1, 1), _erode_iters);
        cv::dilate(mask, mask, kernel, cv::Point(-1, 1), _erode_iters);

        Contours contours;
        Hierarchy hierarchy;
        findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<Ball> balls;
        Contours filtered_contours;
        Hierarchy filtered_hierarchy;
        for (size_t i = 0; i < contours.size(); ++i) {
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contours[i], center, radius);
            if (radius > _min_radius && radius < _max_radius) {
                filtered_contours.push_back(contours[i]);
                filtered_hierarchy.push_back(hierarchy[i]);
                balls.push_back(std::make_tuple(center, radius));
            }
        }

        return std::make_tuple(filtered_contours, filtered_hierarchy, balls);
    }

    void BallDetector::annotate_image(cv::Mat& img, const std::vector<Ball> balls) const
    {
        cv::Scalar bbox_color{0, 255, 255};
        cv::Scalar center_color{0, 0, 255};
        for (const Ball ball : balls) {
            cv::circle(img, std::get<0>(ball), std::get<1>(ball), bbox_color, 3);
            cv::circle(img, std::get<0>(ball), 5, center_color, -1);
        }
    }

    void BallDetector::set_hsv_lower_bound(const cv::Scalar& bound)
    {
        _lb = bound;
    }

    void BallDetector::set_hsv_upper_bound(const cv::Scalar& bound)
    {
        _ub = bound;
    }

    void BallDetector::set_gaussian_kernel_sz(cv::Size sz)
    {
        _gaussian_kernel_sz = sz;
    }

    void BallDetector::set_gaussian_sigma(float sigma)
    {
        _gaussian_sigma = sigma;
    }

    void BallDetector::set_erode_iters(size_t iters)
    {
        _erode_iters = iters;
    }

    void BallDetector::set_dilate_iters(size_t iters)
    {
        _dilate_iters = iters;
    }

    void BallDetector::set_min_radius(float radius)
    {
        _min_radius = radius;
    }

    void BallDetector::set_max_radius(float radius)
    {
        _max_radius = radius;
    }

    cv::Scalar BallDetector::get_hsv_lower_bound() const
    {
        return _lb;
    }

    cv::Scalar BallDetector::get_hsv_upper_bound() const
    {
        return _ub;
    }

    cv::Size BallDetector::get_gaussian_kernel_sz() const
    {
        return _gaussian_kernel_sz;
    }

    float BallDetector::get_gaussian_sigma() const
    {
        return _gaussian_sigma;
    }

    size_t BallDetector::get_erode_iters() const
    {
        return _erode_iters;
    }

    size_t BallDetector::get_dilate_iters() const
    {
        return _dilate_iters;
    }

    float BallDetector::get_min_radius() const
    {
        return _min_radius;
    }

    float BallDetector::get_max_radius() const
    {
        return _max_radius;
    }

} // namespace ros_tp