#include <ros_basics_vision/setup_detector.h>

#include <numeric>

namespace ros_tp {

    SetupDetector::SetupDetector() : SetupDetector(defaults::SetupDetectorParams())
    {
    }

    void SetupDetector::auto_crop_to_setup(cv::Mat& img)
    {
        if (_history_counter % (_num_frames_history + 1) != 0) {
            double area;
            cv::Rect bbox;
            std::tie(bbox, area) = _crop_to_max_contour(img);
            if (area < _min_max_area) {
                _min_max_area = area;
                _min_max_bbox = bbox;
            }

            ++_history_counter;
        }

        img = img(_min_max_bbox);
    }

    std::pair<cv::Rect, double> SetupDetector::_crop_to_max_contour(cv::Mat& img)
    {
        cv::Mat blurred_img;
        cv::GaussianBlur(img, blurred_img, _gaussian_kernel_sz, _gaussian_sigma, _gaussian_sigma);

        cv::Mat bgr_channel[3];
        cv::split(blurred_img, bgr_channel);
        cv::threshold(bgr_channel[0], blurred_img, 80, 255, cv::THRESH_BINARY_INV);

        Contours contours;
        Hierarchy hierarchy;
        findContours(blurred_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        Contour max_contour;
        double max_rect_area = -std::numeric_limits<double>::infinity();
        for (const Contour c : contours) {
            double area = cv::contourArea(c);
            if (area > max_rect_area && (area >= img.rows * img.cols * _setup_area_lb_perc) && (area < img.rows * img.cols * _setup_area_ub_perc)) {
                max_rect_area = area;
                max_contour = c;
            }
        }

        cv::Rect bbox = cv::boundingRect(max_contour);
        return std::make_pair(bbox, max_rect_area);
    }

    void SetupDetector::reset_counter()
    {
        _history_counter = 1;
    }

    void SetupDetector::crop_to_setup(cv::Mat& img, const cv::Rect& bbox) const
    {
        img = img(bbox);
    }

    void SetupDetector::set_gaussian_kernel_sz(cv::Size sz)
    {
        _gaussian_kernel_sz = sz;
    }

    void SetupDetector::set_gaussian_sigma(float sigma)
    {
        _gaussian_sigma = sigma;
    }

    void SetupDetector::set_num_frames_history(int num)
    {
        _num_frames_history = num;
    }

    void SetupDetector::set_setup_area_lb_perc(double perc)
    {
        _setup_area_lb_perc = perc;
    }

    void SetupDetector::set_setup_area_ub_perc(double perc)
    {
        _setup_area_ub_perc = perc;
    }

    double SetupDetector::get_setup_area_lb_perc() const
    {
        return _setup_area_lb_perc;
    }

    double SetupDetector::get_setup_area_ub_perc() const
    {
        return _setup_area_ub_perc;
    }

    cv::Size SetupDetector::get_gaussian_kernel_sz() const
    {
        return _gaussian_kernel_sz;
    }

    float SetupDetector::get_gaussian_sigma() const
    {
        return _gaussian_sigma;
    }

    int SetupDetector::get_num_frames_history() const
    {
        return _num_frames_history;
    }

} // namespace ros_tp