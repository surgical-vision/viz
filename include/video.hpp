#pragma once
#include <opencv2/highgui/highgui.hpp>

namespace viz {

  class VideoIO {

  public:

    VideoIO() : can_read_(false), is_open_(false) {}
    VideoIO(const std::string &inpath);
    VideoIO(const std::string &inpath, const std::string &outpath);
    virtual ~VideoIO();

    cv::Mat Read();
    void Read(cv::Mat &left, cv::Mat &right);

    void Write(const cv::Mat &frame);
    void Write(const cv::Mat &left_frame, const cv::Mat &right_frame);
    void CloseStreams();

    bool CanRead() const { return can_read_; }
    bool IsOpen() const { return is_open_; }

  protected:

    cv::Mat image_input_;
    cv::VideoCapture cap_;
    cv::VideoWriter writer_;
    
    std::size_t image_width_;
    std::size_t image_height_;

    bool can_read_;
    bool is_open_;
  };


}