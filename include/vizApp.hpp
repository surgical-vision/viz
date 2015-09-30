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

#include <cinder/app/AppNative.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Fbo.h>
#include <cinder/gl/Texture.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/MayaCamUI.h>
#include <cinder/params/Params.h>
#include <boost/tuple/tuple.hpp>

#include "camera.hpp"
#include "pose_grabber.hpp"
#include "video.hpp"
#include "sub_window.hpp"

using namespace ci;
using namespace ci::app;

namespace viz {

  /**
  * @struct State
  * @brief Simple container to represent the state of the application.
  */
 
  struct State {

    bool save_all; /**< Flag to save all frames and data. */
    bool save_one; /**< Flag to save the next loaded frame and data. */

    bool load_all; /**< Flag to load all frames and data without stopping. */
    bool load_one; /**< Flag to save just the next frame and data. */

  };

  class vizApp : public AppNative {

  public:

    gl::Fbo shaft_framebuffer;
    gl::Fbo head_framebuffer;
    gl::Fbo clasper1_framebuffer;
    gl::Fbo clasper2_framebuffer;
    gl::Fbo clasper1_base_framebuffer;
    gl::Fbo clasper2_base_framebuffer;

    ci::Vec2i GetEndOfShaft(ci::Vec2f &shaft_start, ci::Vec2f &shaft_end);

    virtual void setup() override;
    virtual void mouseDown(MouseEvent event) override;
    virtual void update() override;
    virtual void draw() override;
    virtual void keyDown(KeyEvent event) override;
    virtual void mouseDrag(MouseEvent event) override;
    virtual void shutdown() override;
    virtual void fileDrop(FileDropEvent event) override;
    virtual void mouseMove(MouseEvent event) override;

    void runVideoButton();
    void editPoseButton(const size_t item_idx);
    void resetViewerButton();
    void savePoseButton();

    static void AddSubWindow(SubWindow *sbw) { sub_windows_.push_back(sbw); }

  protected:

	void drawSegmentation();
	void draw2DTrack();

    SubWindow left_eye;
    SubWindow right_eye;
    SubWindow gui_port;
    SubWindow editor_port;
    SubWindow scene_viewer;
    SubWindow trajectory_viewer;

    State state;
    
    bool running_;

    void draw3DViewports();
    void drawCameraEyes();
    void drawLeftEye();
    void drawRightEye();
    void setupGUI();
    void updateModels();
    void updateVideo();

    /**
    * Create a visualization environment from a configuration file. To see an example configuration file, see config/app.cfg.
    * @param[in] path The path to the config file.
    */
    void setupFromConfig(const std::string &path);
    
    /**
    * Save the state of the current tracked object poses and any windows which are set to save their contents (useful if they have been modified within the GUI).
    */
    void saveState();
    
    /**
    * Save a frame and the current tracked object poses (useful if they have been modified within the GUI).
    */
    void savePoses();

    /**
    * Draw a grid on the ground plane. 
    * @param[in] size The size of the grid.
    * @param[in] step The length of each grid square.
    * @param[in] plane_position The vertical position of the plane.
    */
    void drawGrid(float size = 3.9, float step = 0.3, float plane_position = 0.0);

    /**
    * Call the draw method on all of the trackable targets.
    */
    void drawTargets();

    /**
    * Draw the visualization for either the left or right eye. This corresponds to drawing the camera view and the positions of the instruments in front of the camera.
    * @param[in] texture The background image (i.e. what the camera captured).
    * @param[in] is_left Flag to set whether the camera is the left or right one.
    */
    void drawEye(gl::Texture &texture, bool is_left);
    
    /**
    * Manually edit the pose of the targets in the GUI. This is useful for resolving offsets in the Da Vinci internal pose estimates.
    * @param[in] event The key event which corresponds to an instruction to move the pose in a specific way.
    */
    void movePose(KeyEvent event);

    /**
    * Draw the 3D scene with the camera and trackable targets from a observer viewpoint.
    * @param[in] left_image The current left camera frame, is draw onto the camera model in the 3D viewer.
    * @param[in] right_image The current right camera frame, is draw onto the camera model in the 3D viewer.
    */
    void drawScene(gl::Texture &left_image, gl::Texture &right_image);

    /**
    * Draw the camera view onto the viewport.
    * @param[in] image The camera view.
    */
    void draw2D(gl::Texture &image);

    /** 
    * Draw a 3D model of a camera with it's view mapped onto it's image plane.
    * @param[in] left_image The left image viewed by the camera.
    * @param[in] right_image The right image viewed by the camera.
    */
    void drawCamera(gl::Texture &left_image, gl::Texture &right_image);

    /**
    * Actually map the image onto the camera image plane.
    * @param[in] image_data The image viewed by the camera.
    * @param[in] tl The 3D coordinates of the top left corner of the camera view.
    * @param[in] bl The 3D coordinates of the top bottom corner of the camera view.
    * @param[in] tr The 3D coordinates of the top left right of the camera view.
    * @param[in] br The 3D coordinates of the top bottom right of the camera view.
    */
    void drawImageOnCamera(gl::Texture &image_data, ci::Vec3f &tl, ci::Vec3f &bl, ci::Vec3f &tr, ci::Vec3f &br);

    /**
    * Draw the trajectory of the tracked object as a set of minimal representations of it's pose at each frame.
    * @param[in] transforms The 6 DOF poses the object took at each frame.
    * @param[in] color The color of the trajectory.
    */
    void drawTrajectories(const std::vector<ci::Matrix44f> &transforms, ci::Color &color);

    /**
    * Draw the trajectories of a tracked camera and the ground truth.
    */
    void drawCameraTracker();
    
    /**
    * Load one of the trackables.
    * @param[in] reader The ConfigReader which has the location of the config file for this trackable.
    * @param[in] output_dir_this_run Create a new output directory for the trackables.
    */
    void loadTrackables(const ConfigReader &reader, const std::string &output_dir_this_run);

    /**
    * Wrapper to get the current camera pose. If we have loaded a moveable camera then return its pose, if not then return the identity transform.
    * @return The current camera pose as a 4x4 matrix.
    */
    ci::Matrix44f getCameraPose();

    /**
    * Load a single trackable. Called by loadTrackables().
    * @param[in] filepath The path to the config file for this trackable.
    * @param[in] output_dir The directory where any output for this trackable should be saved.
    */
    void loadTrackable(const std::string &filepath, const std::string &ouput_dir);

    /**
    * Apply a manual offset to a DH parameter camera pose.
    * @param[in] The keyevent which signals which degree of freedom should be changed and whether to increase or decrease its value.
    */
    void applyOffsetToCamera(KeyEvent &event);
    
    /**
    * Apply a manual offset to a DH parameter tracked object pose.
    * @param[in] The keyevent which signals which degree of freedom should be changed and whether to increase or decrease its value.
    */
    void applyOffsetToTrackedObject(KeyEvent &event, const int current_model_idx);

    VideoIO video_left_; /**< The left video IO device. Reads input frames and saves the frames with the corresponding output save on top. */
    VideoIO video_right_;  /**< The right video IO device. Reads input frames and saves the frames with the corresponding output save on top. */
    VideoIO stereo_video_;

    StereoCamera camera_; /**< The physical camera device which models the actual camera which views the scene. Handles projection the models into the image plane of the camera with physically realistic results. */

    gl::Texture left_texture_; /**< The current left camera view */
    gl::Texture right_texture_; /**< The current right camera view */
    gl::Fbo framebuffer_; /**< The framebuffer to hold the drawing for the 'eye' views. */
    gl::Fbo framebuffer_3d_; /**< The framebuffer to the hold the drawing for the 3D view. */

    MayaCamUI maya_cam_2_;
    MayaCamUI maya_cam_; /**< The framebuffer to the hold the drawing for the 3D view. */
    ci::Vec2i	mouse_pos_; /**< Current estimate of mouse position. */

    std::vector< boost::shared_ptr<BasePoseGrabber> > trackables_; /**< The set of trackable objects to draw on the views. */
    boost::shared_ptr<BasePoseGrabber> moveable_camera_; /**< A possibly movable camera too. If this isn't set then the identity camera transform is used (leaving the camera always at the origin). */
    boost::shared_ptr<BasePoseGrabber> tracked_camera_; /**< If we are tracking the possibly moveable camera then we can visualize how the tracking performance was with this object. */
    
    gl::GlslProg shader_; /**< Shader to draw the models more nicely than with the fixed-pipeline drawing. */

    size_t camera_image_width_; /**< The image width we are loading from the camera. */
    size_t camera_image_height_; /**< The image height we are loading from the camera. */
    size_t three_dim_viz_width_; /**< The width of the 3D visualizer window. */
    size_t three_dim_viz_height_; /**< The width of the 3D visualizer window. */

    ci::params::InterfaceGlRef	gui_; /**< The GUI. */

    static std::vector<SubWindow *> sub_windows_; /**< A set of pointers to the various SubWindows in the display. Allows easy iteration over these windows for saving etc. */

    bool reset_viz_port_; /**< Flag to reset the vizport if we move the MayaCam too far away. */

  };


}
