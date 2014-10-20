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

  protected:

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
    void drawTrajectories(std::vector<ci::Vec3f> &points_to_draw, std::vector<ci::Matrix44f> &transforms, ci::Color &color);
    void drawCameraTracker();
    
    void loadTrackables(const ConfigReader &reader);

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
    
    std::vector<BasePoseGrabber> trackables_;
    boost::scoped_ptr<PoseGrabber> moveable_camera_;
    
    gl::GlslProg shader_;


    bool load_next_image_;
    bool save_next_image_;
    bool save_toggle_;

    bool manual_control_toggle_;

    bool has_loaded_new_pwp3d_estimate_;

    bool draw2;
    bool draw3;
    bool draw3d;

  };


}
