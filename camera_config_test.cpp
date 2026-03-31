#include "opencv4/opencv2/opencv.hpp"
#include "include/arg_parser.hpp"

int main(int argc, char **argv){
    ArgParser parser(argc, argv);
    
    std::string device;
    int width, height;

    parser.get<std::string>("camera", device, "/dev/video0", [](const std::string &str){
        return str;
    });

    std::cout<< device<<std::endl;
    return 0;
}