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
    void setup();
    void mouseDown(MouseEvent event);
    void update();
    void draw();
    void keyDown(KeyEvent event);
    void mouseDrag(MouseEvent event);

  protected:

    void saveFrame(gl::Texture texture, bool isLeft);
    void drawGrid(float size = 3.9, float step = 0.3);
    void drawTarget();
    void drawEye(gl::Texture &texture, bool is_left);

    cv::VideoWriter write_left_;
    cv::VideoWriter write_right_;
    std::ofstream ofs_se3_;
    std::ofstream ofs_dh_;

    StereoCamera camera_;

    gl::Texture left_texture_;
    gl::Texture right_texture_;
    gl::Fbo framebuffer_;

    MayaCamUI maya_cam_;
    
    boost::scoped_ptr<ttrk::Handler> handler_;

    boost::scoped_ptr<BasePoseGrabber> camera_pg_;
    ci::Matrix44f camera_pose_;
    
    gl::GlslProg shader_;

    std::vector< boost::tuple <boost::shared_ptr<Trackable>, std::vector<ci::Matrix44f>, std::vector<double> > > moving_objects_pg_;

    //std::vector< ci::Matrix44f > moving_objects_pose_;

    bool load_next_image_;

    bool draw2;
    bool draw3;

  };


}
