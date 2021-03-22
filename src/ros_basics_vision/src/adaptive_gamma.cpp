#include <ros_basics_vision/adaptive_gamma.h>

namespace ros_tp {

    AdaptiveGamma::AdaptiveGamma() : AdaptiveGamma(defaults::AdaptiveGammaParams())
    {
    }

    void AdaptiveGamma::adjust_image(cv::Mat& img)
    {
        if (_iter_counter % _update_rate == 0) {
            compute_bc_gains(img);
            _iter_counter = 0;
        }
        convert(img, _alpha, _beta);
        ++_iter_counter;
    }

    std::pair<float, float> AdaptiveGamma::compute_bc_gains(const cv::Mat& img)
    {
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        cv::Mat hist;
        int channels[] = {0};
        int size[] = {256};
        float hue[] = {0, 180};
        float saturation[] = {0, 256};
        const float* ranges[] = {hue, saturation};
        cv::calcHist(&gray, 1, channels, cv::noArray(), hist, 1, size, ranges);

        float cumulative_dist[hist.rows];
        cumulative_dist[0] = hist.data[0];
        for (size_t i = 1; i < hist.rows; ++i) {
            cumulative_dist[i] = cumulative_dist[i - 1] + hist.data[i];
        }

        float max = cumulative_dist[hist.rows - 1];
        float clip_histogram_perc = _clip_histogram_perc * (max / 100.);
        // clip_histogram_perc /= 2.;

        int min_gray = 0;
        while (cumulative_dist[min_gray] < clip_histogram_perc) {
            ++min_gray;
        }

        int max_gray = hist.rows - 1;
        while (cumulative_dist[max_gray] >= (max - clip_histogram_perc)) {
            --max_gray;
        }

        _alpha = 255. / (max_gray - min_gray);
        _beta = -min_gray * _alpha;

        float brightness = cv::sum(img)[0] / (255 * img.cols * img.rows);
        float ratio = ratio = brightness / _min_brightness;
        _alpha /= ratio;

        return std::make_pair(_alpha, _beta);
    }

    void AdaptiveGamma::convert(cv::Mat& img, float alpha, float beta)
    {
        img = img * alpha + beta;
        img.setTo(0, img < 0);
        img.setTo(255, img > 255);
    }

    void AdaptiveGamma::set_clip_histogram_perc(float perc)
    {
        _clip_histogram_perc = perc;
    }

    void AdaptiveGamma::set_min_brightness(float brightness)
    {
        _min_brightness = brightness;
    }

    void AdaptiveGamma::set_udpate_rate(int rate)
    {
        _update_rate = rate;
    }

    float AdaptiveGamma::get_clip_histogram_perc() const
    {
        return _clip_histogram_perc;
    }

    float AdaptiveGamma::get_min_brightness() const
    {
        return _min_brightness;
    }

    int AdaptiveGamma::get_update_rate() const
    {
        return _update_rate;
    }

} // namespace ros_tp
