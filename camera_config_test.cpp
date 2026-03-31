#include "opencv4/opencv2/opencv.hpp"
#include "include/arg_parser.hpp"

#include <iostream>
#include <string>

namespace {

void PrintUsage() {
    std::cout
        << "Usage: ./camera_config_test "
        << "[--camera=/dev/video0] [--width=1280] [--height=720] [--fps=30] [--buffer=1] "
        << "[--auto_focus=0] [--auto_exposure=1] [--exposure=-6] [--gain=0]"
        << std::endl;
}

void SetAndReport(cv::VideoCapture& cap, int prop, double value, const std::string& name) {
    const bool ok = cap.set(prop, value);
    const double actual = cap.get(prop);
    std::cout << "Set " << name << " = " << value
              << (ok ? " [OK]" : " [FAILED]")
              << ", actual = " << actual << std::endl;
}

}  // namespace

int main(int argc, char **argv){
    ArgParser parser(argc, argv);
    
    std::string device;
    int width = 1280;
    int height = 720;
    int fps = 30;
    int buffer = 1;
    bool auto_focus = true;
    bool auto_exposure = false;
    double exposure = -6.0;
    double gain = 0.0;

    parser.get<std::string>("camera", device, "/dev/video0", ArgParser::to_string);
    parser.get<int>("width", width, 1280, ArgParser::to_int);
    parser.get<int>("height", height, 720, ArgParser::to_int);
    parser.get<int>("fps", fps, 30, ArgParser::to_int);
    parser.get<int>("buffer", buffer, 1, ArgParser::to_int);
    parser.get<bool>("auto_focus", auto_focus, true, ArgParser::to_bool);
    parser.get<bool>("auto_exposure", auto_exposure, false, ArgParser::to_bool);
    parser.get<double>("exposure", exposure, -6.0, ArgParser::to_double);
    parser.get<double>("gain", gain, 0.0, ArgParser::to_double);

    std::cout << "Opening camera: " << device << std::endl;
    cv::VideoCapture cap;
    cap.open(device, cv::CAP_V4L2);

    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera: " << device << std::endl;
        PrintUsage();
        return 1;
    }

    // V4L2 backend usually expects 3 for auto and 1 for manual.
    const double auto_exposure_value = auto_exposure ? 3.0 : 1.0;

    SetAndReport(cap, cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width), "width");
    SetAndReport(cap, cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height), "height");
    SetAndReport(cap, cv::CAP_PROP_FPS, static_cast<double>(fps), "fps");
    SetAndReport(cap, cv::CAP_PROP_BUFFERSIZE, static_cast<double>(buffer), "buffer");
    SetAndReport(cap, cv::CAP_PROP_AUTOFOCUS, auto_focus ? 1.0 : 0.0, "auto_focus");
    SetAndReport(cap, cv::CAP_PROP_AUTO_EXPOSURE, auto_exposure_value, "auto_exposure");

    if (!auto_exposure) {
        SetAndReport(cap, cv::CAP_PROP_EXPOSURE, exposure, "exposure");
    } else {
        std::cout << "Skip manual exposure because auto_exposure is enabled." << std::endl;
    }

    SetAndReport(cap, cv::CAP_PROP_GAIN, gain, "gain");

    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) {
        std::cerr << "Failed to grab a frame after configuration." << std::endl;
        return 1;
    }

    std::cout << "Frame acquired: " << frame.cols << "x" << frame.rows << std::endl;
    std::cout << "Press ESC or q to exit preview." << std::endl;

    while (true) {
        cv::imshow("camera_config_test", frame);
        const int key = cv::waitKey(30);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Camera stream ended or frame grab failed." << std::endl;
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}