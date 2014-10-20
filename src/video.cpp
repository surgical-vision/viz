#include "../include/video.hpp"

using namespace viz;


VideoIO::VideoIO(const std::string &inpath, const std::string &outpath){

  cap_.open(inpath);
  if (!cap_.isOpened()) throw std::runtime_error("Error, could not open input video file");
  image_width_ = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
  image_height_ = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);

  writer_.open(outpath, cv::VideoWriter::fourcc('m', 'j', 'p', 'g'), 25, cv::Size(image_width_, image_height_));

}

VideoIO::~VideoIO(){

  if (cap_.isOpened())
    cap_.release();

  if (writer_.isOpened())
    writer_.release();

}

cv::Mat VideoIO::Read(){

  cv::Mat f;
  cap_ >> f;

  if (f.data == 0x0)
    f = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);

  return f;

}