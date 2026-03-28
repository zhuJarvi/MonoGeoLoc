#include "spdlog/spdlog.h"
#include "include/tic_toc.hpp"
#include "include/serial.hpp"
#include "opencv2/opencv.hpp"

std::string PATH = "../data/video1.mp4";

void groupLines(const std::vector<std::pair<cv::Vec2f, size_t>> &lines, std::vector<std::pair<cv::Vec2f, size_t>> &line_groups, float eps_theta, float eps_rho)
{
    std::vector<std::pair<cv::Vec2f, size_t>> temp;
    for (const auto &line : lines)
    {
        if (temp.empty())
        {
            temp.push_back(line);
        }
        else
        {
            bool found_group = false;
            for (auto &group : temp)
            {
                float group_rho = group.first[0];
                float group_theta = group.first[1];

                if (std::abs(group_theta - line.first[1]) < eps_theta && std::abs(group_rho - line.first[0]) < eps_rho)
                {
                    group.first[0] = (group.first[0] * group.second + line.first[0]) / (group.second + 1);
                    group.first[1] = (group.first[1] * group.second + line.first[1]) / (group.second + 1);
                    group.second += line.second;

                    found_group = true;

                    break;
                }
            }
            if (!found_group)
            {
                temp.push_back(line);
            }
        }
    }
    line_groups = temp;
}

// cv::Vec2f polarLineTransform(const cv::Vec2f &line, const cv::Point &new_origin)
// {
//     float rho = line[0];
//     float theta = line[1];

//     float new_rho = rho - new_origin.x * cos(theta) - new_origin.y * sin(theta);
//     return cv::Vec2f(new_rho, theta);
// }

int main()
{
    // cv::VideoCapture cap(PATH);
    cv::VideoCapture cap(0, cv::CAP_V4L2);

    const std::string dev = "/dev/ttyUSB1";

	int fd = OpenSerialPort(dev, B460800);
    if (fd < 0) {
        spdlog::error("Failed to open serial port");
        return 1;
    }
    else{
        spdlog::info("Serial port opened successfully");
    }

    if(!cap.isOpened()) {
        spdlog::error("Failed to open video");
        return 1;
    }
    else{
        spdlog::info("Video opened successfully");
    }

    while (cap.isOpened())
    {
        TicToc tic_toc;
        tic_toc.tic();
        cv::Mat frame;
        cap >> frame;
        if (frame.empty())
            break;

        cv::Mat gussian_blur;
        cv::GaussianBlur(frame, gussian_blur, cv::Size(5, 5), 0);

        cv::Mat hsv_mat;
        cv::cvtColor(gussian_blur, hsv_mat, cv::COLOR_BGR2HSV);

        cv::Mat yellow_mask;
        cv::inRange(hsv_mat, cv::Scalar(20, 150, 150), cv::Scalar(30, 255, 255), yellow_mask);

        cv::Mat eroded_mask;
        cv::erode(yellow_mask, eroded_mask, cv::Mat(), cv::Point(-1, -1), 2);

        cv::Mat dilated_mask;
        cv::dilate(eroded_mask, dilated_mask, cv::Mat(), cv::Point(-1, -1), 2);

        cv::Mat canny;
        cv::Canny(dilated_mask, canny, 50, 150);

        cv::dilate(canny, canny, cv::Mat(), cv::Point(-1, -1), 2);

        // 计算质心，直线转换到质心坐标系进行过滤

        cv::Point mass_center = cv::Point(0, 0);
        cv::Moments moments = cv::moments(canny, true);
        if (moments.m00 > 0)
        {
            int cx = static_cast<int>(moments.m10 / moments.m00);
            int cy = static_cast<int>(moments.m01 / moments.m00);
            mass_center = cv::Point(cx, cy);
            cv::circle(frame, mass_center, 5, cv::Scalar(255, 0, 0), -1);
        }
        float mass_theta = atan2(mass_center.y, mass_center.x);
        float mass_rho = sqrt(mass_center.x * mass_center.x + mass_center.y * mass_center.y);

        std::vector<cv::Vec2f> lines;
        cv::HoughLines(canny, lines, 1, CV_PI / 180, 70);

        std::vector<std::pair<cv::Vec2f, size_t>> line_groups;
        for (const auto &line : lines)
        {
            // 坐标变换
            float new_rho = line[0] - mass_rho * cos(line[1] - mass_theta);
            float new_theta = line[1];
            if (new_rho < 0)
            {
                new_rho = -new_rho;
                new_theta -= CV_PI;
            }

            line_groups.push_back({cv::Vec2f(new_rho, new_theta), 1});
        }
        float eps_theta = 0.5, eps_rho = 20;

        groupLines(line_groups, line_groups, eps_theta, eps_rho);
        std::sort(line_groups.begin(), line_groups.end(), [](const std::pair<cv::Vec2f, size_t> &a, const std::pair<cv::Vec2f, size_t> &b)
                  { return a.first[1] > b.first[1]; });
        std::vector<cv::Vec2f> clean_lines;

        float orign_theta = atan2(-mass_center.y, -mass_center.x);
        float orign_rho = mass_rho;
        for (const auto &group : line_groups)
        {
            float rho = group.first[0];
            float theta = group.first[1];

            float new_rho = rho - orign_rho * cos(theta - orign_theta);
            float new_theta = theta;
            if (new_rho < 0)
            {
                new_rho = -new_rho;
                new_theta -= CV_PI;
            }

            clean_lines.push_back(cv::Vec2f(new_rho, new_theta));

            // 画线
            // float a = cos(new_theta), b = sin(new_theta);
            // float x0 = a * new_rho, y0 = b * new_rho;
            // cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
            // cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
            // cv::line(frame, pt1, pt2, cv::Scalar(0, 0, 255), 2);
        }

        // 计算交点
        std::vector<cv::Vec2f> intersections;

        for (size_t i = 0; i < clean_lines.size(); ++i)
        {
            const auto &line1 = clean_lines[i];
            const auto &line2 = clean_lines[(i + 1) % clean_lines.size()];

            float rho1 = line1[0], theta1 = line1[1];
            float rho2 = line2[0], theta2 = line2[1];

            float x, y;
            float denom = sin(theta2 - theta1);
            if (std::abs(denom) < 1e-7)
            {
                spdlog::warn("Lines are nearly parallel, skipping intersection calculation for lines {} and {}", i, (i + 1) % clean_lines.size());
                continue;
            }
            x = (rho1 * sin(theta2) - rho2 * sin(theta1)) / denom;
            y = (rho2 * cos(theta1) - rho1 * cos(theta2)) / denom;
            intersections.emplace_back(x, y);
            spdlog::info("rho1 = {:.2f}, theta1 = {:.2f}, rho2 = {:.2f}, theta2 = {:.2f}", rho1, theta1, rho2, theta2);
            spdlog::info("Intersection {}: x = {:.2f}, y = {:.2f}", i, x, y);
        }

        // 校正为矩形
        std::vector<cv::Vec2f> rect_points;
        float length_buf = 0;
        if (intersections.size() <= 4)
        {
            rect_points = intersections;
        }
        else
        {
            for (size_t skip = 0; skip < intersections.size(); ++skip)
            {
                rect_points.clear();
                length_buf = 0;
                for (size_t i = 0; i < intersections.size(); ++i)
                {
                    if (i == skip)
                        continue;

                    const auto &pt1 = intersections[i];
                    size_t next_index = (i + 1) % intersections.size();
                    if (next_index == skip)
                        next_index = (next_index + 1) % intersections.size();
                    const auto &pt2 = intersections[next_index];

                    cv::Vec2f vec = pt2 - pt1;

                    float length = cv::norm(vec);
                    if (length_buf == 0 || (length < 1.4  * length_buf && length > 0.6 * length_buf))
                    {
                        length_buf = length;
                        rect_points.push_back(pt1);
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
        if (rect_points.size() != 4)
        {
            spdlog::error("Rectangle point size = {}", rect_points.size());
        }

        for (const auto &pt : intersections)
        {
            cv::circle(frame, static_cast<cv::Point>(pt), 5, cv::Scalar(0, 255, 0), -1);
        }

        for (const auto &pt : rect_points)
        {
            cv::circle(frame, static_cast<cv::Point>(pt), 5, cv::Scalar(0, 0, 255), -1);
        }

        spdlog::info("Detected {} lines, grouped into {} groups", lines.size(), line_groups.size());

        double elapsed_time = tic_toc.toc();
        spdlog::info("time cost : {:.5f} ms fps: {:.2f}", elapsed_time, 1000.0 / elapsed_time);
        cv::imshow("Detected Rectangles", frame);

        cv::imshow("Yellow Mask", canny);
        cv::waitKey(100);


        // std::vector<uint8_t> rx_buf = {0x59, 0x74, 0x}

    }
}