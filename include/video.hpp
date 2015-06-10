#pragma once

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

#include <opencv2/highgui/highgui.hpp>

namespace viz {

  /**
  * @class VideoIO
  * @brief Simple video input output wrapper.
  * This class wraps up video input and output in a simple to use container. Interfaces to videos or image files.
  */
  class VideoIO {

  public:

    /**
    * Set up a default object which basically does nothing. Only useful for delayed opening.
    */
    VideoIO() : can_read_(false), is_open_(false) {}

    /**
    * Open a input only version of the class - when we don't necessarily want to write anything.
    * @param[in] inpath The path to the input video file or image file.
    */
    explicit VideoIO(const std::string &inpath);
    
    /**
    * Open an input and output file.
    * @param[in] inpath The path to the input video file or image file.
    * @param[in] outpath The path to the output video file.
    */
    VideoIO(const std::string &inpath, const std::string &outpath);

    /**
    * Close the video streams.
    */
    virtual ~VideoIO();

    /**
    * Read the next frame. 
    * @return An empty matrix if not enabled or a black frame if there's nothing else to read.
    */
    cv::Mat Read();

    /**
    * Read the next when we have a packed (side-by-side) stereo frame. Returns an empty matrix if not enabled or a black frame if there's nothing else to read.
    * @param[out] The left part of the frame we read.
    * @param[out] The right part of the frame we read.
    */
    void Read(cv::Mat &left, cv::Mat &right);

    /**
    * Write the current frame.
    * @param[in] The current frame. Resizes it if's the wrong size.
    */
    void Write(const cv::Mat &frame);
    
    /**
    * Write the current stereo to a a packed (side-by-side) stereo frame. 
    * @param[in] The left part of the frame we want to write.
    * @param[in] The right part of the frame we want to write.
    */
    void Write(const cv::Mat &left_frame, const cv::Mat &right_frame);

    /**
    * Close the video streams.
    */
    void CloseStreams();


    /**
    * Check if we can read from this file.
    * @return True if we can, false otherwise.
    */
    bool CanRead() const { return can_read_; }

    /**
    * Check if this file is open.
    * @return True if it is, false otherwise.
    */
    bool IsOpen() const { return is_open_; }

  protected:

    cv::Mat image_input_; /**< If we read from an image file, it's stored here. */
    cv::VideoCapture cap_; /**< Video capture interface. */
    cv::VideoWriter writer_; /**< Video writer interface. */
    
    std::size_t image_width_; /**< The image width we are writing. */
    std::size_t image_height_; /**< The image height we are writing. */

    bool can_read_; /**< Boolean for whether we can actually read this file. */
    bool is_open_; /**< Boolean for whether we have opened the file. */

  };


}