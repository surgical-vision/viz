#pragma once

#include <cinder/app/AppNative.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Fbo.h>
#include <cinder/gl/Texture.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/MayaCamUI.h>
#include "camera.hpp"
#include <utils/handler.hpp>
#include "pose_grabber.hpp"
#include "trackable.hpp"
#include <boost/tuple/tuple.hpp>

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
    void drawGrid(float size = 3.9, float step = 0.3);
    void drawTarget();
    void drawEye(gl::Texture &texture, bool is_left);
    void savePoseAsSE3(std::ofstream &ofs, const ci::Matrix44d &camera_pose, const Pose &pose);
    void savePoseAsSE3AndDH(std::ofstream &ofs, const ci::Matrix44d &camera_pose, const Pose &pose);
    void draw3D(); 
    void drawTarget(ci::Matrix44f &inverse);

    cv::VideoWriter write_left_;
    cv::VideoWriter write_right_;
    //std::ofstream ofs_se3_;
    //std::ofstream ofs_dh_;

    StereoCamera camera_;

    gl::Texture left_texture_;
    gl::Texture right_texture_;
    gl::Fbo framebuffer_;

    MayaCamUI maya_cam_;
    
    boost::scoped_ptr<ttrk::Handler> handler_;

    boost::scoped_ptr<BasePoseGrabber> camera_pg_;
    boost::scoped_ptr<BasePoseGrabber> camera_estimates_;

    ci::Matrix44f camera_estimate_matrix_;
    ci::Matrix44f camera_pose_;
    std::ofstream ofs_cam_;

    gl::GlslProg shader_;

    std::vector< boost::shared_ptr<Trackable> > moving_objects_pg_;
    
    //std::vector< ci::Matrix44f > moving_objects_pose_;

    bool load_next_image_;
    bool save_next_image_;
    bool save_toggle_;

    bool draw2;
    bool draw3;
    bool draw3d;

  };


}
