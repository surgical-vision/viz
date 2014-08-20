#pragma once

#include <cinder/app/AppNative.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Fbo.h>
#include <cinder/gl/Texture.h>
#include <cinder/MayaCamUI.h>
#include "camera.hpp"
#include <utils/handler.hpp>
#include "pose_grabber.hpp"
#include "trackable.hpp"

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

    void drawGrid(float size = 39, float step = 3);
    void drawTarget();
    void drawEye(gl::Texture &texture, bool is_left);

    StereoCamera camera_;

    gl::Texture left_texture_;
    gl::Texture right_texture_;
    gl::Fbo framebuffer_;

    MayaCamUI maya_cam_;
    
    boost::scoped_ptr<ttrk::Handler> handler_;

    boost::scoped_ptr<BasePoseGrabber> camera_pg_;
    ci::Matrix44f camera_pose_;
        
    std::vector< std::pair<boost::shared_ptr<Trackable>, ci::Matrix44f> > moving_objects_pg_;
    //std::vector< ci::Matrix44f > moving_objects_pose_;

    bool load_next_image_;

  };


}
