#pragma once
#include <opencv2/highgui.hpp>

namespace viz {

  class VideoIO {

  public:

    VideoIO() {}
    VideoIO(const std::string &inpath, const std::string &outpath);
    virtual ~VideoIO();

    cv::Mat Read();

  protected:

    cv::VideoCapture cap_;
    cv::VideoWriter writer_;
    
    std::size_t image_width_;
    std::size_t image_height_;

  };


}