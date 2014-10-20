#include "../include/video.hpp"

using namespace viz;


VideoIO::VideoIO(const std::string &inpath, const std::string &outpath){

  cap_.open(inpath);
  if (!cap_.isOpened()) throw std::runtime_error("Error, could not open input video file");
  const int width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
  const int height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);

  writer_.open(outpath, cv::VideoWriter::fourcc('m', 'j', 'p', 'g'), 25, cv::Size(width, height));

}