#pragma once

#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "simple_mcl/particle.hpp"

namespace simple_mcl {

class Visualizer
{
public:
    Visualizer(double min_x, double max_x, double min_y, double max_y,
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

    bool worldToImage(double wx, double wy, int *u, int *v) const
    {
        if (wx < min_x_ || wx > max_x_ || wy < min_y_ || wy > max_y_) return false;

        int col = static_cast<int>((wx - min_x_) * render_scale_);
        int row = static_cast<int>((max_y_ - wy) * render_scale_);
        if (u) *u = col;
        if (v) *v = row;

        return true;
    }

    cv::Mat drawFrame(const std::vector<Particle> &particles,
        const Pose &truth, const Pose &estimate,
        const std::vector<Landmark> &landmarks) const
    {
        cv::Mat frame = base_image_.clone();

        for (const auto &lm : landmarks) {
            int u = 0;
            int v = 0;
            if (worldToImage(lm.x, lm.y, &u, &v)) {
                cv::circle(frame, cv::Point(u, v), 2, cv::Scalar(0, 0, 0), -1);
            }
        }
        for (const auto &p : particles) {
            int u = 0;
            int v = 0;
            if (worldToImage(p.pose.x, p.pose.y, &u, &v)) {
                cv::circle(frame, cv::Point(u, v), 0, cv::Scalar(255, 0, 0), -1);
            }
        }

        return frame;
    }

private:
    cv::Mat base_image_;
    double min_x_{0.0};
    double max_x_{0.0};
    double min_y_{0.0};
    double max_y_{0.0};
    int render_scale_{1};
    int max_width_{800};
    int max_height_{800};
};

}  // namespace simple_mcl
