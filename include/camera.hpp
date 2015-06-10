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

#include <opencv2/opencv.hpp>
#include <string>
#include <cinder/gl/gl.h>
#include <cinder/Matrix.h>
#include <cinder/MayaCamUI.h>
#include <cinder/gl/Light.h>

namespace viz {

  /**
  * @class Camera
  * @brief Simple OpenGL friendly camera.
  * A simple class to handle drawing with a calibrated camera in an OpenGL environment. As we are working in surgical vision there is a light attached to each camera. Although not super realistic, it's easiest thing to do without calibrating the light position.
  */

  class Camera {

  public:

    /**
    * Default camera constructor. Sets an ID for the light and initilizes it as a directional light source.
    * @param[in] light_id The light ID.
    */
    explicit Camera(int light_id) : is_setup_(false), light_(ci::gl::Light(ci::gl::Light::Type::DIRECTIONAL, light_id)) {}

    /**
    * Setup a camera from the standard calibration parameters.
    * @param[in] camera_matrix The camera matrix containing focal length and principal point.
    * @param[in] distortion_params The zhang polynomial model for camera distortion.
    * @param[in] image_width The x image resolution of the camera.
    * @param[in] image_height The y image resolution of the camera.
    * @param[in] near_clip The OpenGL near clip plane.
    * @param[in] far_clip The OpenGL far clip plane.
    */
    void Setup(const cv::Mat camera_matrix, const cv::Mat distortion_params, const int image_width, const int image_height, const int near_clip_distance, const int far_clip_distance);

    /**
    * Set the GL_PROJECTION matrix so that this camera is the one we are using. 
    */
    void makeCurrentCamera() const ;

    /**
    * Quick accessor for the image width.
    * @return The image width.
    */
    int getImageWidth() const { return image_width_; }
    
    /**
    * Quick accessor for the image height.
    * @return The image height.
    */    
    int getImageHeight() const { return image_height_; }

    /**
    * Get a handle to this camera's light.
    * @return The light source.
    */
    ci::gl::Light &getLight() { return light_; }

    /**
    * Switch on the light source.
    */
    void TurnOnLight();
    
    /**
    * Switch off the light source.
    */
    void TurnOffLight();

  protected:

    cv::Mat camera_matrix_; /**< The camera calibration matrix. */
    ci::Matrix44f gl_projection_matrix_; /**< The GL_PROJECTIONMATRIX for this camera calibration. Ignores distortion. */
    cv::Mat distortion_params_; /**< The camera distortion parameters. */

    int image_width_; /**< The x resolution of the camera image. */
    int image_height_; /**< The y resolution of the camera image. */
    int near_clip_distance_; /**< The near clip plane for OpenGL. */
    int far_clip_distance_; /**< The far clip plane for OpenGL.  */

    bool is_setup_; /**< Flag for whether the camera calibration is loaded. */
     
    ci::gl::Light light_; /**< A cinder wrapper for an OpenGL light. */

  };

  /**
  * @class StereoCamera
  * @brief Simple OpenGL friendly stereo camera.
  * A simple class to wrap 2 Camera objects in a single StereoCamera.
  */
  class StereoCamera {

  public:

    /**
    * Default constructor creating a left and right eye.
    */
    StereoCamera() : left_eye_(0), right_eye_(1) {}


    /**
    * Setup a camera from the standard calibration parameters using an OpenCV XML file.
    * @param[in] calibration_file An OpenCV calibration file.
    * @param[in] near_clip The OpenGL near clip plane.
    * @param[in] far_clip The OpenGL far clip plane.
    */
    void Setup(const std::string &calibration_file, const int near_clip_distance, const int far_clip_distance); 

    /**
    * Move the GL_MODELVIEW to the left camera position and setup the GL_VIEWPORT.
    * @param[in] cam A Cinder GL 'MayaCam' which is used to wrap up the data about this camera.
    * @param[in] current_camera_pose The world coordinates of the stereo camera eye.
    */
    void setupLeftCamera(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose);
    
    /**
    * Move the GL_MODELVIEW to the right camera position and setup the GL_VIEWPORT.
    * @param[in] cam A Cinder GL 'MayaCam' which is used to wrap up the data about this camera.
    * @param[in] current_camera_pose The world coordinates of the stereo camera eye.
    */
    void setupRightCamera(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose);
    
    /**
    * Set the GL_PROJECTION matrix so the left camera is the one we are rendering with.
    */
    void makeLeftEyeCurrent();
    
    /**
    * Set the GL_PROJECTION matrix so that the right camera is the one we are rendering with.
    */
    void makeRightEyeCurrent();

    /**
    * Reset the viewport.
    */
    void unsetCameras();

    /**
    * Get a handle to this stereo camera's light (which is the left camera light as this represents a stereo endoscope).
    * @return The left eye's light.
    */
    ci::gl::Light &getLight() { return left_eye_.getLight(); }

    /**
    * Accessor to get the extrinsic translation between the cameras.
    * @return the translation between the camera coordinates.
    */
    ci::Vec3f getExtrinsicTranslation() const { return extrinsic_translation_; }
    
    /**
    * Accessor to get the extrinsic rotation between the cameras.
    * @return the rotation between the camera coordinates.
    */
    ci::Matrix33f getExtrinsicRotation() const { return extrinsic_rotation_; }

    /**
    * Switch on the left eye's light.
    */
    void TurnOnLight();
    
    /**
    * Switch off the left eye's light.
    */
    void TurnOffLight();

    /**
    * Accessor to get the left eye of the rig.
    * @return the left camera.
    */
    const Camera &GetLeftCamera() { return left_eye_; }

    /**
    * Accessor to get the right eye of the rig.
    * @return the right camera.
    */
    const Camera &GetRightCamera() { return right_eye_; }

  protected:

    /**
    * Move the GL_MODELVIEW to correspond to the left camera and also move its light too.
    * @param[in] cam A Cinder GL 'MayaCam' which is used to wrap up the data about this camera.
    * @param[in] current_camera_pose The world coordinates of the stereo camera eye.
    */
    void moveEyeToLeftCam(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose);

    /**
    * Move the GL_MODELVIEW to correspond to the right camera and also move its light too.
    * @param[in] cam A Cinder GL 'MayaCam' which is used to wrap up the data about this camera.
    * @param[in] current_camera_pose The world coordinates of the stereo camera eye.
    */
    void moveEyeToRightCam(ci::MayaCamUI &cam, const ci::Matrix44f &current_camera_pose);
    
    /**
    * Modifies a Bouguet camera calibration to make it compatible with OpenGL as they use different coordinate systems.
    * @param[in,out] left_camera_matrix The camera matrix of the left camera calibrated using J.Y. Bouguet's toolbox. After function call it is set up for use in OpenGL.
    * @param[in,out] right_camera_matrix The camera matrix of the right camera calibrated using J.Y. Bouguet's toolbox. After function call it is set up for use in OpenGL.
    * @param[in,out] extrinsic_rotation The extrinsic rotation computed by J.Y. Bouguet's toolbox. After function call it is set up for use in OpenGL.
    * @param[in,out] extrinsic_rotation The extrinsic translation computed by J.Y. Bouguet's toolbox. After function call it is set up for use in OpenGL.
    * @param[in] image_width Image width needed to transform the principal points.
    * @param[in] image_height Image height needed to transform the principal points.
    */
    void convertBouguetToGLCoordinates(cv::Mat &left_camera_matrix, cv::Mat &right_camera_matrix, cv::Mat &extrinsic_rotation, cv::Mat &extrinsic_translation, const int image_width, const int image_height);

    Camera left_eye_; /**< The camera corresponding to the stereo rig's left eye. */
    Camera right_eye_; /**< The camera corresponding to the stereo rig's right eye. */

    GLint viewport_cache_[4]; /**< Cache of the viewport (when we change it for the eyes so it's not lost). */

    ci::Matrix33f extrinsic_rotation_; /**< Rotation between the eye's coordinates system. If it's in Bouguet format this is rotation matrix which transforms points in left eye coordinate to right eye coordiantes. If GL then it's transformation that transforms coordinates system from left to right (i.e. the inverse of the Bouguet one). */
    ci::Vec3f extrinsic_translation_; /**< Translation between the eye's coordinates system. If it's in Bouguet format this is translation vector which transforms points in left eye coordinate to right eye coordiantes. If GL then it's transformation that transforms coordinates system from left to right (i.e. the inverse of the Bouguet one). * */

  };


}