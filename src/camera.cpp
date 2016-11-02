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

#include "../include/camera.hpp"
#include <cinder/gl/gl.h>
#include <cinder/app/App.h>


using namespace viz;

void Camera::Setup(const cv::Mat camera_matrix, const cv::Mat distortion_params, const int image_width, const int image_height, const int near_clip_distance, const int far_clip_distance){

  image_width_ = image_width;
  image_height_ = image_height;
  near_clip_distance_ = near_clip_distance;
  far_clip_distance_ = far_clip_distance;

  //load opencv camera calibration parameters
  camera_matrix_ = camera_matrix.clone();
  distortion_params_ = distortion_params.clone();

  //setup openGL projection matrix
  gl_projection_matrix_.setToNull();
  gl_projection_matrix_.m00 = (float)camera_matrix_.at<double>(0, 0);
  gl_projection_matrix_.m11 = (float)camera_matrix_.at<double>(1, 1);
  gl_projection_matrix_.m02 = (float)-camera_matrix_.at<double>(0, 2);
  gl_projection_matrix_.m12 = (float)-camera_matrix_.at<double>(1, 2);
  gl_projection_matrix_.m22 = (float)(near_clip_distance + far_clip_distance);
  gl_projection_matrix_.m23 = (float)(near_clip_distance * far_clip_distance);
  gl_projection_matrix_.m32 = -1;

  is_setup_ = true;

}

void Camera::makeCurrentCamera() const {

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, image_width_, 0, image_height_, near_clip_distance_, far_clip_distance_);
  glMultMatrixf(gl_projection_matrix_.m);

}

void StereoCamera::Setup(const std::string &calibration_filename, const int near_clip_distance, const int far_clip_distance){

  if(!boost::filesystem::exists(boost::filesystem::path(calibration_filename)))
    throw(std::runtime_error("Error, could not find camera calibration file: " + calibration_filename + "\n"));

  cv::FileStorage fs;

  try{

    cv::Mat l_intrinsic, l_distortion;
    cv::Mat r_intrinsic, r_distortion;
    fs.open(calibration_filename, cv::FileStorage::READ);

    fs["Left_Camera_Matrix"] >> l_intrinsic;
    fs["Left_Distortion_Coefficients"] >> l_distortion;
    fs["Right_Camera_Matrix"] >> r_intrinsic;
    fs["Right_Distortion_Coefficients"] >> r_distortion;

    cv::Mat rotation(3, 3, CV_64FC1), translation(3, 1, CV_64FC1);
    fs["Extrinsic_Camera_Rotation"] >> rotation;
    fs["Extrinsic_Camera_Translation"] >> translation;

    cv::Mat image_size;
    fs["Image_Dimensions"] >> image_size;
    size_t image_width = image_size.at<int>(0);
    size_t image_height = image_size.at<int>(1);

    convertBouguetToGLCoordinates(l_intrinsic, r_intrinsic, rotation, translation, image_width, image_height);

    left_eye_.Setup(l_intrinsic, l_distortion, image_width, image_height, near_clip_distance, far_clip_distance);
    right_eye_.Setup(r_intrinsic, r_distortion, image_width, image_height, near_clip_distance, far_clip_distance);

    for (int r = 0; r<rotation.rows; r++){
      for (int c = 0; c<rotation.cols; c++){
        extrinsic_rotation_.at(r,c) = (float)rotation.at<double>(r, c);
      }
      extrinsic_translation_[r] = (float)translation.at<double>(r, 0);
    }

  }
  catch (cv::Exception& e){

    std::cerr << "Error while reading from camara calibration file.\n" << e.msg << "\n";
    throw std::runtime_error("");

  }

}

void StereoCamera::convertBouguetToGLCoordinates(cv::Mat &left_camera_matrix, cv::Mat &right_camera_matrix, cv::Mat &extrinsic_rotation, cv::Mat &extrinsic_translation, const int image_width, const int image_height){

  //first flip the principal points
  left_camera_matrix.at<double>(1, 2) = image_height - left_camera_matrix.at<double>(1, 2);
  right_camera_matrix.at<double>(1, 2) = image_height - right_camera_matrix.at<double>(1, 2);

  extrinsic_rotation = extrinsic_rotation.inv();
  extrinsic_translation = extrinsic_translation * -1;

}

void StereoCamera::setupLeftCamera(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose){

  glGetIntegerv(GL_VIEWPORT, viewport_cache_);
  glViewport(0, 0, left_eye_.getImageWidth(), left_eye_.getImageHeight());
  moveEyeToLeftCam(cam, current_camera_pose);

}

void StereoCamera::setupRightCamera(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose){

  glViewport(0, 0, right_eye_.getImageWidth(), right_eye_.getImageHeight());
  moveEyeToRightCam(cam, current_camera_pose);

}


void StereoCamera::unsetCameras(){

  glViewport(viewport_cache_[0], viewport_cache_[1], viewport_cache_[2], viewport_cache_[3]);
  glDisable(GL_LIGHTING);

}

void StereoCamera::makeLeftEyeCurrent(){

  left_eye_.makeCurrentCamera();

}

void StereoCamera::makeRightEyeCurrent(){

  right_eye_.makeCurrentCamera();

}

void StereoCamera::TurnOnLight(){

  ci::Vec3f eye_point(0, 0, 0);
  ci::Vec3f view_direction(0, 0, 1);
  ci::Vec3f world_up(0, -1, 0);
  
  glEnable(GL_LIGHTING);

  left_eye_.getLight().setPosition(eye_point);
  left_eye_.getLight().lookAt(eye_point, view_direction);
  left_eye_.getLight().enable();

}

void StereoCamera::TurnOffLight(){

  glDisable(GL_LIGHTING);

}

void StereoCamera::moveEyeToLeftCam(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose){

  ci::CameraPersp camP;

  ci::Vec3f eye_point(0, 0, 0);
  ci::Vec3f view_direction(0, 0, 1);
  ci::Vec3f world_up(0, -1, 0);

  camP.setEyePoint(eye_point);
  camP.setViewDirection(view_direction);
  camP.setWorldUp(world_up);

  cam.setCurrentCam(camP);

  ci::gl::setMatrices(cam.getCamera());

  glEnable(GL_LIGHTING);

  left_eye_.getLight().setPosition(eye_point);
  left_eye_.getLight().lookAt(eye_point, view_direction);
  left_eye_.getLight().enable();

  ci::gl::multModelView(current_camera_pose.inverted());
  
  left_eye_.makeCurrentCamera();
}

void StereoCamera::moveEyeToRightCam(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose){

  ci::CameraPersp camP;
  
  ci::Vec3f eye_point(0, 0, 0);
  ci::Vec3f view_direction(0, 0, 1);
  ci::Vec3f world_up(0, -1, 0);
  

  glEnable(GL_LIGHTING);

  left_eye_.getLight().setPosition(eye_point);
  left_eye_.getLight().lookAt(eye_point, view_direction);
  left_eye_.getLight().enable();

  camP.setEyePoint(extrinsic_translation_);
  camP.setViewDirection(extrinsic_rotation_ * view_direction);
  camP.setWorldUp(extrinsic_rotation_ * world_up);

  cam.setCurrentCam(camP);

  ci::gl::setMatrices(cam.getCamera());

  ci::gl::multModelView(current_camera_pose.inverted());

  right_eye_.makeCurrentCamera();

}
