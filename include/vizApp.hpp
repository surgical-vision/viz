#pragma once

#include <cinder/app/AppNative.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Fbo.h>
#include <cinder/gl/Texture.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/MayaCamUI.h>
#include <boost/tuple/tuple.hpp>

#include "camera.hpp"
#include "pose_grabber.hpp"
#include "video.hpp"


using namespace ci;
using namespace ci::app;

namespace viz {

  class vizApp : public AppNative {

  public:
    virtual void setup() override;
    virtual void mouseDown(MouseEvent event) override;
    virtual void update() override;
    virtual void draw() override;
    virtual void keyDown(KeyEvent event) override;
    virtual void mouseDrag(MouseEvent event) override;
    virtual void shutdown() override;
    virtual void fileDrop(FileDropEvent event) override;

  protected:

    void setupFromConfig(const std::string &path);
    
    void saveFrame(gl::Texture texture, bool isLeft);
    void drawGrid(float size = 3.9, float step = 0.3, float plane_position = 0.0);
    void drawTarget();
    void drawEye(gl::Texture &texture, bool is_left);
    //void savePoseAsSE3(std::ofstream &ofs, const Pose &camera_pose, const Pose &pose);
    //void savePoseAsSE3AndDH(std::ofstream &ofs, const Pose &camera_pose, const Pose &pose);
    //void saveDH(std::ofstream &suj_ofs, std::ofstream &j_ofs, const Pose &pose);
    void draw3D(gl::Texture &image);
    void draw2D(gl::Texture &image);

    void drawTarget(ci::Matrix44f &inverse);
    void drawCamera(gl::Texture &image_data);
    void drawImageOnCamera(gl::Texture &image_data, ci::Vec3f &tl, ci::Vec3f &bl, ci::Vec3f &tr, ci::Vec3f &br);
    void drawTrajectories(const std::vector<ci::Matrix44f> &transforms, ci::Color &color);
    void drawCameraTracker();
    
    void loadTrackables(const ConfigReader &reader);
    ci::Matrix44f getCameraPose();
    void loadTrackable(const std::string &filepath);

    void applyOffsetToCamera(KeyEvent &event);
    void applyOffsetToTrackedObject(KeyEvent &event, boost::shared_ptr<DHDaVinciPoseGrabber> grabber);

    VideoIO video_left_; 
    VideoIO video_right_;  
    
    StereoCamera camera_;

    gl::Texture left_texture_;
    gl::Texture right_texture_;
    gl::Fbo framebuffer_;
    gl::Fbo framebuffer_3d_;

    MayaCamUI maya_cam_;
    
    std::vector< boost::shared_ptr<BasePoseGrabber> > trackables_;
    boost::scoped_ptr<BasePoseGrabber> moveable_camera_;
    boost::scoped_ptr<BasePoseGrabber> tracked_camera_;
    
    gl::GlslProg shader_;

    size_t camera_image_width_; /**< The image width we are loading from the camera. */
    size_t camera_image_height_; /**< The image height we are loading from the camera. */
    size_t three_dim_viz_width_; /**< The width of the 3D visualizer window. */
    size_t three_dim_viz_height_; /**< The width of the 3D visualizer window. */

    bool run_video_; /**< Toggle to set whether the video just runs and loads new poses without prompting. */
    bool load_next_image_; /**< Toggle to manually load the next frame and poses. */
    bool save_toggle_; /**< Toggle to set saving of all data from the visualizer (poses and video frames). */
    bool update_toggle_; /**< Toggle to set whether the updates load from the file or just refresh. Useful for instance if manual offsets are being applied to pose of trackables in the UI and you want to draw these new poses. */


  };


}
