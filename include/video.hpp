#pragma once
#include <opencv2/highgui.hpp>

namespace viz {

  class VideoIO {

  public:

    VideoIO() : can_read_(false) {}
    VideoIO(const std::string &inpath, const std::string &outpath);
    virtual ~VideoIO();

    cv::Mat Read();
    void Write(const cv::Mat &frame);
    void CloseStreams();

    bool CanRead() const { return can_read_; }

  protected:

    cv::VideoCapture cap_;
    cv::VideoWriter writer_;
    
    std::size_t image_width_;
    std::size_t image_height_;

    bool can_read_;

  };


}