#pragma once

#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "simple_mcl/particle.hpp"

namespace simple_mcl {

class Visualizer
{
public:
    Visualizer()
    {
        int width = static_cast<int>(std::ceil((max_x_ - min_x_) * render_scale_));
        int height = static_cast<int>(std::ceil((max_y_ - min_y_) * render_scale_));
        base_image_ = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    }

private:

};

}  // namespace simple_mcl
