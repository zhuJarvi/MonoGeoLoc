#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
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

struct CameraConfig {
	int width = 1280;
	int height = 720;
	double fps = 30.0;
	int buffer_size = 1;
	bool auto_focus = false;
	std::optional<bool> auto_exposure;
	std::optional<double> exposure;
	std::optional<double> gain;
};

bool ParseIntOption(const std::string& arg, const std::string& key, int* out) {
	const std::string prefix = "--" + key + "=";
	if (arg.rfind(prefix, 0) != 0) {
		return false;
	}
	try {
		*out = std::stoi(arg.substr(prefix.size()));
	} catch (...) {
		return false;
	}
	return true;
}

bool ParseDoubleOption(const std::string& arg, const std::string& key, double* out) {
	const std::string prefix = "--" + key + "=";
	if (arg.rfind(prefix, 0) != 0) {
		return false;
	}
	try {
		*out = std::stod(arg.substr(prefix.size()));
	} catch (...) {
		return false;
	}
	return true;
}

bool ParseBoolOption(const std::string& arg, const std::string& key, bool* out) {
	const std::string prefix = "--" + key + "=";
	if (arg.rfind(prefix, 0) != 0) {
		return false;
	}

	const std::string value = arg.substr(prefix.size());
	if (value == "1" || value == "true" || value == "on") {
		*out = true;
		return true;
	}
	if (value == "0" || value == "false" || value == "off") {
		*out = false;
		return true;
	}
	return false;
}

bool ParseCameraConfig(int argc, char** argv, CameraConfig* cfg) {
	for (int i = 2; i < argc; ++i) {
		const std::string arg = argv[i];
		int int_value = 0;
		double double_value = 0.0;
		bool bool_value = false;

		if (ParseIntOption(arg, "width", &int_value)) {
			cfg->width = int_value;
			continue;
		}
		if (ParseIntOption(arg, "height", &int_value)) {
			cfg->height = int_value;
			continue;
		}
		if (ParseDoubleOption(arg, "fps", &double_value)) {
			cfg->fps = double_value;
			continue;
		}
		if (ParseIntOption(arg, "buffer", &int_value)) {
			cfg->buffer_size = int_value;
			continue;
		}
		if (ParseBoolOption(arg, "auto_focus", &bool_value)) {
			cfg->auto_focus = bool_value;
			continue;
		}
		if (ParseBoolOption(arg, "auto_exposure", &bool_value)) {
			cfg->auto_exposure = bool_value;
			continue;
		}
		if (ParseDoubleOption(arg, "exposure", &double_value)) {
			cfg->exposure = double_value;
			continue;
		}
		if (ParseDoubleOption(arg, "gain", &double_value)) {
			cfg->gain = double_value;
			continue;
		}

		std::cerr << "Unknown camera option: " << arg << std::endl;
		return false;
	}

	if (cfg->width <= 0 || cfg->height <= 0 || cfg->fps <= 0.0 || cfg->buffer_size <= 0) {
		std::cerr << "Invalid camera options: width/height/fps/buffer must be positive." << std::endl;
		return false;
	}

	return true;
}

void ApplyCameraConfig(cv::VideoCapture& cap, const CameraConfig& cfg) {
	cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(cfg.width));
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(cfg.height));
	cap.set(cv::CAP_PROP_FPS, cfg.fps);
	cap.set(cv::CAP_PROP_BUFFERSIZE, static_cast<double>(cfg.buffer_size));
	cap.set(cv::CAP_PROP_AUTOFOCUS, cfg.auto_focus ? 1.0 : 0.0);

	const bool request_manual_exposure = cfg.exposure.has_value() || cfg.gain.has_value();
	if (cfg.auto_exposure.has_value()) {
		if (cfg.auto_exposure.value()) {
			// Different backends use different value ranges for CAP_PROP_AUTO_EXPOSURE.
			cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1.0);
			cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
		} else {
			cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.0);
			cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
		}
	} else if (request_manual_exposure) {
		// If manual values are provided, try to switch to manual mode first.
		cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.0);
		cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
	}

	if (cfg.exposure.has_value()) {
		cap.set(cv::CAP_PROP_EXPOSURE, cfg.exposure.value());
	}
	if (cfg.gain.has_value()) {
		cap.set(cv::CAP_PROP_GAIN, cfg.gain.value());
	}

	const double applied_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
	const double applied_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
	const double applied_fps = cap.get(cv::CAP_PROP_FPS);
	const double applied_buffer = cap.get(cv::CAP_PROP_BUFFERSIZE);
	const double applied_af = cap.get(cv::CAP_PROP_AUTOFOCUS);
	const double applied_ae = cap.get(cv::CAP_PROP_AUTO_EXPOSURE);
	const double applied_exposure = cap.get(cv::CAP_PROP_EXPOSURE);
	const double applied_gain = cap.get(cv::CAP_PROP_GAIN);

	std::cout << "Camera config requested: "
			  << "width=" << cfg.width << ", "
			  << "height=" << cfg.height << ", "
			  << "fps=" << cfg.fps << ", "
			  << "buffer=" << cfg.buffer_size << ", "
			  << "auto_focus=" << (cfg.auto_focus ? "on" : "off") << ", "
			  << "auto_exposure="
			  << (cfg.auto_exposure.has_value() ? (cfg.auto_exposure.value() ? "on" : "off") : "keep") << ", "
			  << "exposure=" << (cfg.exposure.has_value() ? cv::format("%.3f", cfg.exposure.value()) : "keep") << ", "
			  << "gain=" << (cfg.gain.has_value() ? cv::format("%.3f", cfg.gain.value()) : "keep") << std::endl;

	std::cout << "Camera config applied: "
			  << "width=" << applied_width << ", "
			  << "height=" << applied_height << ", "
			  << "fps=" << applied_fps << ", "
			  << "buffer=" << applied_buffer << ", "
			  << "auto_focus=" << applied_af << ", "
			  << "auto_exposure=" << applied_ae << ", "
			  << "exposure=" << applied_exposure << ", "
			  << "gain=" << applied_gain << std::endl;
}

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
	const bool camera_mode = IsNumber(src);
	CameraConfig camera_cfg;

	if (camera_mode && !ParseCameraConfig(argc, argv, &camera_cfg)) {
		std::cerr << "Usage: ./main_node [camera_id] [--width=1280 --height=720 --fps=30 --buffer=1 --auto_focus=0 --auto_exposure=1 --exposure=-6 --gain=0]"
				  << std::endl;
		return 1;
	}

	if (camera_mode) {
		cap.open(std::stoi(src));
		if (cap.isOpened()) {
			ApplyCameraConfig(cap, camera_cfg);
		}
	} else {
		cap.open(src);
		if (!cap.isOpened()) {
			single_image = cv::imread(src);
			image_mode = !single_image.empty();
		}
	}

	if (!cap.isOpened() && !image_mode) {
		std::cerr << "Cannot open source: " << src << std::endl;
		std::cerr << "Usage: ./main_node [camera_id|video_path|image_path] [--width=1280 --height=720 --fps=30 --buffer=1 --auto_focus=0 --auto_exposure=1 --exposure=-6 --gain=0]"
				  << std::endl;
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
