#pragma once

#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "simple_mcl/particle.hpp"

namespace simple_mcl {

class Visualizer
{
public:
    Visualizer(
        double min_x, double max_x, double min_y, double max_y,
        int render_scale = 10, int max_width = 800, int max_height = 800)
        : min_x_(min_x),
          max_x_(max_x),
          min_y_(min_y),
          max_y_(max_y),
          render_scale_(render_scale),
          max_width_(max_width),
          max_height_(max_height)
    {
        int width = static_cast<int>(std::ceil((max_x_ - min_x_) * render_scale_));
        int height = static_cast<int>(std::ceil((max_y_ - min_y_) * render_scale_));
        base_image_ = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    }

private:

};

}  // namespace simple_mcl
