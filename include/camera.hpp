#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <cinder/gl/gl.h>
#include <cinder/Matrix.h>
#include <cinder/MayaCamUI.h>
#include <cinder/gl/Light.h>

namespace viz {

  class Camera {

  public:

    Camera(int light_id) : is_setup_(false), light_(ci::gl::Light(ci::gl::Light::Type::DIRECTIONAL, light_id)) {}

    void Setup(const cv::Mat camera_matrix, const cv::Mat distortion_params, const int image_width, const int image_height, const int near_clip_distance, const int far_clip_distance);

    void makeCurrentCamera() const ;

    int getImageWidth() const { return image_width_; }
    int getImageHeight() const { return image_height_; }

    ci::gl::Light &getLight() { return light_; }

    void TurnOnLight();
    void TurnOffLight();

  protected:

    cv::Mat camera_matrix_;
    ci::Matrix44f gl_projection_matrix_;
    cv::Mat distortion_params_;

    int image_width_;
    int image_height_;
    int near_clip_distance_;
    int far_clip_distance_;

    bool is_setup_;
    
    ci::gl::Light light_;

  };


  class StereoCamera {

  public:

    StereoCamera() : left_eye_(0), right_eye_(1) {}

    void Setup(const std::string &calibration_file, const int image_width, const int image_height, const int near_clip_distance, const int far_clip_distance); 

    void setupLeftCamera(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose);
    void setupRightCamera(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose);
    
    void makeLeftEyeCurrent();
    void makeRightEyeCurrent();

    void unsetCameras();

    ci::gl::Light &getLight() { return left_eye_.getLight(); }

    void convertBouguetToGLCoordinates_3d(cv::Mat &rotation, cv::Mat &translation);

    ci::Vec3f getExtrinsicTranslation() const { return extrinsic_translation_; }
    ci::Matrix33f getExtrinsicRotation() const { return extrinsic_rotation_; }

    void TurnOnLight();
    void TurnOffLight();

    const Camera &GetLeftCamera() { return left_eye_; }
    const Camera &GetRightCamera() { return right_eye_; }

  protected:

    void moveEyeToLeftCam(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose);
    void moveEyeToRightCam(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose);
    
    void convertBouguetToGLCoordinates(cv::Mat &left_camera_matrix, cv::Mat &right_camera_matrix, cv::Mat &extrinsic_rotation, cv::Mat &extrinsic_translation, const int image_width, const int image_height);
    void convertBouguetToDaVinciCoordinates(cv::Mat &left_camera_matrix, cv::Mat &right_camera_matrix, cv::Mat &extrinsic_rotation, cv::Mat &extrinsic_translation, const int image_width, const int image_height);

    Camera left_eye_;
    Camera right_eye_;

    GLint viewport_cache_[4];

    ci::Matrix33f extrinsic_rotation_;
    ci::Vec3f extrinsic_translation_;

  };


}