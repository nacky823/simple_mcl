// SPDX-FileCopyrightText: 2026 Yuki NAGAKI youjiyongmu4@gmail.com
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cmath>
#include <cstdio>
#include <string>

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
        const std::vector<Landmark> &landmarks, double time_sec) const
    {
        cv::Mat frame = base_image_.clone();

        drawAxes(frame);
        cv::putText(frame, "t=" + formatTime(time_sec) + " [s]", cv::Point(40, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 50, 50), 1);

        for (const auto &lm : landmarks) {
            int u = 0;
            int v = 0;
            if (worldToImage(lm.x, lm.y, &u, &v)) {
                cv::circle(frame, cv::Point(u, v), 2, cv::Scalar(0, 0, 0), -1);
            }
        }
        for (std::size_t i = 0; i < landmarks.size(); ++i) {
            int u = 0;
            int v = 0;
            if (worldToImage(landmarks[i].x, landmarks[i].y, &u, &v)) {
                cv::putText(frame, "L" + std::to_string(i + 1),
                            cv::Point(u + 4, v - 4),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
            }
        }
        for (const auto &p : particles) {
            int u = 0;
            int v = 0;
            if (worldToImage(p.pose.x, p.pose.y, &u, &v)) {
                cv::circle(frame, cv::Point(u, v), 1, cv::Scalar(255, 0, 0), -1);
            }
        }

        int tu = 0;
        int tv = 0;
        if (worldToImage(truth.x, truth.y, &tu, &tv)) {
            cv::circle(frame, cv::Point(tu, tv), 4, cv::Scalar(0, 255, 0), -1);
            int hx = static_cast<int>(tu + 8 * std::cos(truth.theta));
            int hy = static_cast<int>(tv - 8 * std::sin(truth.theta));
            cv::arrowedLine(frame, cv::Point(tu, tv), cv::Point(hx, hy), cv::Scalar(0, 200, 0), 1);
        }
        int eu = 0;
        int ev = 0;
        if (worldToImage(estimate.x, estimate.y, &eu, &ev)) {
            cv::circle(frame, cv::Point(eu, ev), 4, cv::Scalar(0, 0, 255), -1);
            int hx = static_cast<int>(eu + 8 * std::cos(estimate.theta));
            int hy = static_cast<int>(ev - 8 * std::sin(estimate.theta));
            cv::arrowedLine(frame, cv::Point(eu, ev), cv::Point(hx, hy), cv::Scalar(0, 0, 200), 1);
        }

        const int width = frame.cols;
        const int height = frame.rows;
        double scale_x = static_cast<double>(max_width_) / static_cast<double>(width);
        double scale_y = static_cast<double>(max_height_) / static_cast<double>(height);
        double scale = std::min(scale_x, scale_y);
        if (scale < 1.0) {
            cv::Mat scaled;
            cv::resize(frame, scaled, cv::Size(), scale, scale, cv::INTER_AREA);
            return scaled;
        }

        return frame;
    }

private:
    void drawAxes(cv::Mat &frame) const
    {
        const int tick_step = 5;
        const cv::Scalar tick_color(120, 120, 120);
        for (int x = static_cast<int>(std::ceil(min_x_));
             x <= static_cast<int>(std::floor(max_x_)); x += tick_step) {
            int u = 0;
            int v0 = 0;
            int v1 = 0;
            if (worldToImage(static_cast<double>(x), min_y_, &u, &v0) &&
                worldToImage(static_cast<double>(x), min_y_ + 0.5, &u, &v1)) {
                cv::line(frame, cv::Point(u, v0), cv::Point(u, v1), tick_color, 1);
                cv::putText(frame, std::to_string(x), cv::Point(u + 2, v0 - 2),
                            cv::FONT_HERSHEY_SIMPLEX, 0.35, tick_color, 1);
            }
        }
        for (int y = static_cast<int>(std::ceil(min_y_));
             y <= static_cast<int>(std::floor(max_y_)); y += tick_step) {
            int u0 = 0;
            int u1 = 0;
            int v = 0;
            if (worldToImage(min_x_, static_cast<double>(y), &u0, &v) &&
                worldToImage(min_x_ + 0.5, static_cast<double>(y), &u1, &v)) {
                cv::line(frame, cv::Point(u0, v), cv::Point(u1, v), tick_color, 1);
                cv::putText(frame, std::to_string(y), cv::Point(u0 + 2, v - 2),
                            cv::FONT_HERSHEY_SIMPLEX, 0.35, tick_color, 1);
            }
        }
        int ux = 0;
        int vx = 0;
        if (worldToImage(max_x_, min_y_, &ux, &vx)) {
            cv::putText(frame, "x", cv::Point(ux - 12, vx - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(30, 30, 30), 1);
        }
        int uy = 0;
        int vy = 0;
        if (worldToImage(min_x_, max_y_, &uy, &vy)) {
            cv::putText(frame, "y", cv::Point(uy + 6, vy + 16),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(30, 30, 30), 1);
        }
    }

    std::string formatTime(double time_sec) const
    {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.2f", time_sec);
        return std::string(buf);
    }

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
