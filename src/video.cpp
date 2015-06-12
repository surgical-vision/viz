/**

viz - A robotics visualizer specialized for the da Vinci robotic system.
Copyright (C) 2014 Max Allan

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

**/

#include "../include/video.hpp"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <cinder/app/App.h>

using namespace viz;

VideoIO::VideoIO(const std::string &inpath){

   if (boost::filesystem::path(inpath).extension().string() == ".png" ||
    boost::filesystem::path(inpath).extension().string() == ".jpg" ||
    boost::filesystem::path(inpath).extension().string() == ".jpeg" ||
    boost::filesystem::path(inpath).extension().string() == ".bmp"){
    image_input_ = cv::imread(inpath);
    
    image_width_ = image_input_.cols;
    image_height_ = image_input_.rows;

  }
  else{
    cap_.open(inpath);
    if (!cap_.isOpened()) throw std::runtime_error("Error, could not open input video file");
    image_width_ = cap_.get(CV_CAP_PROP_FRAME_WIDTH);
    image_height_ = cap_.get(CV_CAP_PROP_FRAME_HEIGHT);
  }

  can_read_ = true;
  is_open_ = true;

}


VideoIO::VideoIO(const std::string &inpath, const std::string &outpath) : VideoIO(inpath) {

  if (boost::filesystem::path(inpath).extension().string() == ".png" ||
    boost::filesystem::path(inpath).extension().string() == ".jpg" ||
    boost::filesystem::path(inpath).extension().string() == ".jpeg" ||
    boost::filesystem::path(inpath).extension().string() == ".bmp"){
    image_input_ = cv::imread(inpath);

    image_width_ = image_input_.cols;
    image_height_ = image_input_.rows;


  }
  else{
    cap_.open(inpath);
    if (!cap_.isOpened()) throw std::runtime_error("Error, could not open input video file");
    image_width_ = cap_.get(CV_CAP_PROP_FRAME_WIDTH);
    image_height_ = cap_.get(CV_CAP_PROP_FRAME_HEIGHT);
  } 

  writer_.open(outpath, CV_FOURCC('M','J','P','G'), 25, cv::Size(image_width_, image_height_));

  if (!writer_.isOpened()) throw std::runtime_error("Error, could not open output video file");

  can_read_ = true;
  is_open_ = true;

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

  if (cap_.isOpened()){
    cap_ >> f;
  }
  else if (!image_input_.empty()){
    f = image_input_.clone();
  }

  if (f.data == 0x0){
    f = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);
    can_read_ = false;
  }

  return f;

}

void VideoIO::Read(cv::Mat &left, cv::Mat &right){

  if (!can_read_) {
    left = cv::Mat::zeros(cv::Size(0, 0), CV_8UC3);
    right = cv::Mat::zeros(cv::Size(0, 0), CV_8UC3);
  }

  cv::Mat f;
  cap_ >> f;

  if (f.data == 0x0){
    left = cv::Mat::zeros(cv::Size(image_width_/2, image_height_), CV_8UC3);
    right = cv::Mat::zeros(cv::Size(image_width_/2, image_height_), CV_8UC3);
    can_read_ = false;
  }

  cv::Mat l = f(cv::Rect(0, 0, f.cols / 2, f.rows));
  l.copyTo(left);

  cv::Mat r = f(cv::Rect(f.cols / 2, 0, f.cols / 2, f.rows));
  r.copyTo(right);

}

void VideoIO::Write(const cv::Mat &frame){

  if (frame.size() != cv::Size(image_width_, image_height_)){
    cv::Mat resized_frame;
    cv::resize(frame, resized_frame, cv::Size(image_width_, image_height_));
    writer_ << resized_frame;
  }
  else{
    writer_ << frame;
  }


}


void VideoIO::Write(const cv::Mat &left_frame, const cv::Mat &right_frame){

  cv::Mat frame(image_height_, image_width_, CV_8UC3);
  cv::Mat lf = frame(cv::Rect(0, 0, left_frame.cols, left_frame.rows));
  cv::Mat rf = frame(cv::Rect(left_frame.cols, 0, left_frame.cols, frame.rows));
  left_frame.copyTo(lf);
  right_frame.copyTo(rf);
  writer_ << frame;

}