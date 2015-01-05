#include "../include/video.hpp"
#include <opencv2/highgui/highgui_c.h>

using namespace viz;


VideoIO::VideoIO(const std::string &inpath, const std::string &outpath){

  cap_.open(inpath);
  if (!cap_.isOpened()) throw std::runtime_error("Error, could not open input video file");
  image_width_ = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
  image_height_ = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);

  
  writer_.open(outpath, CV_FOURCC('D','I','B',' '), 25, cv::Size(image_width_, image_height_));

  if (!writer_.isOpened()) throw std::runtime_error("Error, could not open output video file");

  can_read_ = true;

}

VideoIO::~VideoIO(){

  CloseStreams();

}

void VideoIO::CloseStreams(){

  if (cap_.isOpened())
    cap_.release();

  if (writer_.isOpened())
    writer_.release();


}

cv::Mat VideoIO::Read(){

  if (!can_read_) return cv::Mat::zeros(cv::Size(0, 0), CV_8UC3);

  cv::Mat f;
  cap_ >> f;

  if (f.data == 0x0){
    f = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);
    can_read_ = false;
  }

  return f;

}

void VideoIO::Write(const cv::Mat &frame){

  writer_ << frame;

}