#pragma once
#include <opencv2/highgui.hpp>

namespace viz {

  class VideoIO {

  public:

    VideoIO() {}
    VideoIO(const std::string &inpath, const std::string &outpath);

  protected:

    cv::VideoCapture cap_;
    cv::VideoWriter writer_;

  };


}