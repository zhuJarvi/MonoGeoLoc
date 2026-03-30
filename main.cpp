#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

struct YellowRectDetection {
	std::vector<cv::Point2f> corners;
	cv::RotatedRect min_rect;
	double score = 0.0;
};

std::vector<cv::Point2f> OrderCornersClockwise(const std::vector<cv::Point2f>& pts) {
	std::vector<cv::Point2f> ordered = pts;
	if (ordered.size() != 4) {
		return {};
	}

	cv::Point2f center(0.0f, 0.0f);
	for (const auto& p : ordered) {
		center += p;
	}
	center *= 0.25f;

	std::sort(ordered.begin(), ordered.end(), [&](const cv::Point2f& a, const cv::Point2f& b) {
		const float ang_a = std::atan2(a.y - center.y, a.x - center.x);
		const float ang_b = std::atan2(b.y - center.y, b.x - center.x);
		return ang_a < ang_b;
	});

	// Rotate order so that index 0 is top-left to stabilize downstream usage.
	int top_left_idx = 0;
	float min_sum = ordered[0].x + ordered[0].y;
	for (int i = 1; i < 4; ++i) {
		const float s = ordered[i].x + ordered[i].y;
		if (s < min_sum) {
			min_sum = s;
			top_left_idx = i;
		}
	}
	std::rotate(ordered.begin(), ordered.begin() + top_left_idx, ordered.end());
	return ordered;
}

double MeanGradientOnSegment(const cv::Mat& grad_mag, const cv::Point2f& a, const cv::Point2f& b) {
	constexpr int kSamples = 20;
	double sum = 0.0;
	int valid = 0;

	for (int i = 0; i <= kSamples; ++i) {
		const float t = static_cast<float>(i) / static_cast<float>(kSamples);
		const float x = a.x + t * (b.x - a.x);
		const float y = a.y + t * (b.y - a.y);

		const int xi = std::clamp(static_cast<int>(std::round(x)), 0, grad_mag.cols - 1);
		const int yi = std::clamp(static_cast<int>(std::round(y)), 0, grad_mag.rows - 1);

		sum += grad_mag.at<float>(yi, xi);
		++valid;
	}

	return (valid > 0) ? (sum / static_cast<double>(valid)) : 0.0;
}

double ScoreQuadrilateral(const std::vector<cv::Point>& contour,
						  const std::vector<cv::Point2f>& corners,
						  const cv::Mat& grad_mag,
						  cv::RotatedRect* out_min_rect) {
	if (corners.size() != 4) {
		return -1.0;
	}

	const double contour_area = std::fabs(cv::contourArea(contour));
	if (contour_area < 500.0) {
		return -1.0;
	}

	const cv::RotatedRect min_rect = cv::minAreaRect(corners);
	const double rect_area = min_rect.size.area();
	if (rect_area < 1e-6) {
		return -1.0;
	}

	const double fill_ratio = contour_area / rect_area;
	if (fill_ratio < 0.60) {
		return -1.0;
	}

	const double w = std::max(min_rect.size.width, min_rect.size.height);
	const double h = std::min(min_rect.size.width, min_rect.size.height);
	if (h < 1e-6) {
		return -1.0;
	}

	const double aspect = w / h;
	if (aspect > 6.0) {
		return -1.0;
	}

	double edge_score = 0.0;
	for (int i = 0; i < 4; ++i) {
		edge_score += MeanGradientOnSegment(grad_mag, corners[i], corners[(i + 1) % 4]);
	}
	edge_score /= 4.0;

	// AprilTag-style candidate quality: geometry consistency + edge contrast.
	const double score = 0.002 * contour_area + 2.0 * fill_ratio + 0.015 * edge_score;
	if (out_min_rect != nullptr) {
		*out_min_rect = min_rect;
	}
	return score;
}

std::optional<YellowRectDetection> DetectYellowRectangle(const cv::Mat& frame_bgr, cv::Mat* out_mask) {
	if (frame_bgr.empty()) {
		return std::nullopt;
	}

	cv::Mat blurred;
	cv::GaussianBlur(frame_bgr, blurred, cv::Size(5, 5), 0.0);

	cv::Mat hsv;
	cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

	// Yellow in HSV (OpenCV H range is [0, 179]).
	cv::Mat mask;
	cv::inRange(hsv, cv::Scalar(15, 80, 80), cv::Scalar(40, 255, 255), mask);

	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
	cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

	if (out_mask != nullptr) {
		*out_mask = mask.clone();
	}

	cv::Mat gray;
	cv::cvtColor(frame_bgr, gray, cv::COLOR_BGR2GRAY);

	cv::Mat gx, gy;
	cv::Sobel(gray, gx, CV_32F, 1, 0, 3);
	cv::Sobel(gray, gy, CV_32F, 0, 1, 3);
	cv::Mat grad_mag;
	cv::magnitude(gx, gy, grad_mag);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	std::optional<YellowRectDetection> best;

	for (const auto& contour : contours) {
		if (contour.size() < 4) {
			continue;
		}

		const double perimeter = cv::arcLength(contour, true);
		if (perimeter < 50.0) {
			continue;
		}

		std::vector<cv::Point> approx;
		cv::approxPolyDP(contour, approx, 0.02 * perimeter, true);
		if (approx.size() != 4 || !cv::isContourConvex(approx)) {
			continue;
		}

		std::vector<cv::Point2f> corners(4);
		for (int i = 0; i < 4; ++i) {
			corners[i] = approx[i];
		}
		corners = OrderCornersClockwise(corners);
		if (corners.size() != 4) {
			continue;
		}

		cv::RotatedRect min_rect;
		const double score = ScoreQuadrilateral(contour, corners, grad_mag, &min_rect);
		if (score < 0.0) {
			continue;
		}

		if (!best.has_value() || score > best->score) {
			best = YellowRectDetection{corners, min_rect, score};
		}
	}

	return best;
}

bool IsNumber(const std::string& s) {
	if (s.empty()) {
		return false;
	}
	return std::all_of(s.begin(), s.end(), [](unsigned char ch) { return std::isdigit(ch) != 0; });
}

void DrawDetection(cv::Mat& frame, const YellowRectDetection& det) {
	for (int i = 0; i < 4; ++i) {
		cv::line(frame, det.corners[i], det.corners[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
		cv::circle(frame, det.corners[i], 4, cv::Scalar(0, 0, 255), -1);
		cv::putText(frame,
					std::to_string(i),
					det.corners[i] + cv::Point2f(5.0f, -5.0f),
					cv::FONT_HERSHEY_SIMPLEX,
					0.5,
					cv::Scalar(255, 0, 0),
					1,
					cv::LINE_AA);
	}

	cv::Point2f center = det.min_rect.center;
	cv::putText(frame,
				"yellow rectangle",
				center + cv::Point2f(8.0f, -8.0f),
				cv::FONT_HERSHEY_SIMPLEX,
				0.7,
				cv::Scalar(0, 255, 255),
				2,
				cv::LINE_AA);

	cv::putText(frame,
				"score=" + cv::format("%.2f", det.score),
				center + cv::Point2f(8.0f, 16.0f),
				cv::FONT_HERSHEY_SIMPLEX,
				0.6,
				cv::Scalar(0, 255, 255),
				2,
				cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
	std::string src = (argc > 1) ? argv[1] : "0";

	cv::VideoCapture cap;
	cv::Mat single_image;
	bool image_mode = false;

	if (IsNumber(src)) {
		cap.open(std::stoi(src));
	} else {
		cap.open(src);
		if (!cap.isOpened()) {
			single_image = cv::imread(src);
			image_mode = !single_image.empty();
		}
	}

	if (!cap.isOpened() && !image_mode) {
		std::cerr << "Cannot open source: " << src << std::endl;
		std::cerr << "Usage: ./main_node [camera_id|video_path|image_path]" << std::endl;
		return 1;
	}

	if (image_mode) {
		cv::Mat mask;
		auto det = DetectYellowRectangle(single_image, &mask);
		if (det.has_value()) {
			DrawDetection(single_image, det.value());
		}

		cv::imshow("yellow_rect_detection", single_image);
		cv::imshow("yellow_mask", mask);
		std::cout << "Press any key to exit." << std::endl;
		cv::waitKey(0);
		return 0;
	}

	cv::Mat frame;
	while (cap.read(frame)) {
		cv::Mat mask;
		auto det = DetectYellowRectangle(frame, &mask);

		if (det.has_value()) {
			DrawDetection(frame, det.value());
		}

		cv::imshow("yellow_rect_detection", frame);
		cv::imshow("yellow_mask", mask);

		const int key = cv::waitKey(100);
		if (key == 27 || key == 'q' || key == 'Q') {
			break;
		}
	}

	return 0;
}
