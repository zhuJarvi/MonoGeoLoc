#include "opencv2/opencv.hpp"

int main(){

    cv::VideoCapture cap(0); // 打开默认摄像头
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
    cap.set(cv::CAP_PROP_EXPOSURE, 100);

    while(cap.isOpened()) {
        cv::Mat frame;
        cap >> frame; // 从摄像头捕获一帧

        if (frame.empty()) {
            std::cerr << "无法捕获视频帧" << std::endl;
            break;
        }

        cv::imshow("Camera Feed", frame); // 显示捕获的帧

        if (cv::waitKey(30) >= 0) { // 等待30ms，按任意键退出
            break;
        }
    }
    return 0;

}