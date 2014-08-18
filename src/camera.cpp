#include "../include/camera.hpp"

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
  gl_projection_matrix_.m00 = camera_matrix_.at<float>(0, 0);
  gl_projection_matrix_.m11 = camera_matrix_.at<float>(1, 1);
  gl_projection_matrix_.m02 = -camera_matrix_.at<float>(0, 2);
  gl_projection_matrix_.m12 = -camera_matrix_.at<float>(1, 2);
  gl_projection_matrix_.m22 = near_clip_distance + far_clip_distance;
  gl_projection_matrix_.m23 = near_clip_distance * far_clip_distance;
  gl_projection_matrix_.m32 = -1;

  is_setup_ = true;

}

void Camera::makeCurrentCamera() const {

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, image_width_, 0, image_height_, near_clip_distance_, far_clip_distance_);
  glMultMatrixf(gl_projection_matrix_.m);

}

void StereoCamera::Setup(const std::string &calibration_filename, const int image_width, const int image_height, const int near_clip_distance, const int far_clip_distance){

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

    left_eye_.Setup(l_intrinsic, l_distortion, image_width, image_height, near_clip_distance, far_clip_distance);
    right_eye_.Setup(r_intrinsic, r_distortion, image_width, image_height, near_clip_distance, far_clip_distance);

    for (int r = 0; r<rotation.rows; r++){
      for (int c = 0; c<rotation.cols; c++){
        extrinsic_rotation_.at(r,c) = rotation.at<double>(r, c);
      }
      extrinsic_translation_[r] = translation.at<double>(r, 0);
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
  left_camera_matrix.at<double>(1, 2) = image_height - left_camera_matrix.at<double>(1, 2);

  //set the rotation matrix
  cv::Mat inv_rotation = extrinsic_rotation.inv();
  cv::Mat flip = cv::Mat::eye(3, 3, CV_64FC1);// [1, 0, 0; 0, -1, 0; 0, 0, -1];
  flip.at<double>(1, 1) = -1; flip.at<double>(2, 2) = -1;
  cv::Mat in_gl_coords = flip * inv_rotation * flip;
  extrinsic_rotation = in_gl_coords.clone();

  //set the translation matrix by flipping x and z
  extrinsic_translation.at<double>(0, 0) *= -1;
  extrinsic_translation.at<double>(2, 0) *= -1;
  
}


void StereoCamera::setupCameras(){

  glGetIntegerv(GL_VIEWPORT, viewport_);
  glViewport(0, 0, left_eye_.getImageWidth(), left_eye_.getImageHeight());

}


void StereoCamera::unsetCameras(){

  glViewport(viewport_[0], viewport_[1], viewport_[2], viewport_[3]);

}

void StereoCamera::makeLeftEyeCurrent(){

  left_eye_.makeCurrentCamera();

}

void StereoCamera::makeRightEyeCurrent(){

  right_eye_.makeCurrentCamera();

}


void StereoCamera::moveEyeToLeftCam(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose){

  ci::CameraPersp camP;
  ci::Quatf rotation(current_camera_pose.subMatrix33(0, 0));

  ci::Vec3f eye_point = current_camera_pose.getTranslate().xyz();
  ci::Vec3f view_direction = rotation * ci::Vec3f(0, 0, -1);
  ci::Vec3f world_up = rotation * ci::Vec3f(0, 1, 0);

  camP.setEyePoint(eye_point);
  camP.setViewDirection(view_direction);
  camP.setWorldUp(world_up);
  cam.setCurrentCam(camP);

}

void StereoCamera::moveEyeToRightCam(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose){

  ci::CameraPersp camP;
  ci::Quatf rotation(current_camera_pose.subMatrix33(0, 0));
  ci::Quatf extrinsic_rotation(extrinsic_rotation_);

  ci::Vec3f eye_point = current_camera_pose.getTranslate().xyz();
  ci::Vec3f view_direction = rotation * ci::Vec3f(0, 0, -1);
  ci::Vec3f world_up = rotation * ci::Vec3f(0, 1, 0);

  camP.setEyePoint(eye_point + (rotation*extrinsic_translation_));
  camP.setViewDirection(ci::Quatf(extrinsic_rotation_) * view_direction);
  camP.setWorldUp(ci::Quatf(extrinsic_rotation_) * world_up);
  cam.setCurrentCam(camP);

}
