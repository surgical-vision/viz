#include "../include/vizApp.hpp"

using namespace viz;

const int WINDOW_WIDTH = 736;
const int WINDOW_HEIGHT = 288;

void vizApp::setup(){

  const std::string input_video("f.avi");
  const std::string output_video("g.avi");

  handler_.reset(new ttrk::VideoHandler(input_video,output_video));

  setWindowSize(2 * WINDOW_WIDTH, WINDOW_HEIGHT);

}

void vizApp::mouseDown( MouseEvent event ){
}

void vizApp::update(){
}

void vizApp::draw(){
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) ); 

  framebuffer_.bindFramebuffer();
  drawEye(left_texture_, true);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf(0, framebuffer_.getHeight(), framebuffer_.getWidth(), 0));

  framebuffer_.bindFramebuffer();
  drawEye(right_texture_, false);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf(framebuffer_.getWidth(), framebuffer_.getHeight(), 2 * framebuffer_.getWidth(), 0));

}

void vizApp::drawTarget(){

}


void vizApp::drawEye(gl::Texture &texture, bool is_left){

  gl::clear(Color(0, 0, 0));

  gl::disableDepthRead();

  gl::draw(texture);

  camera_.setupCameras();

  gl::pushMatrices();

  if (is_left){
    camera_.moveEyeToLeftCam(maya_cam_);
  }
  else{
    camera_.moveEyeToRightCam(maya_cam_);
  }

  gl::setMatrices(maya_cam_.getCamera());

  if (is_left){
    camera_.makeLeftEyeCurrent();
  }
  else{
    camera_.makeRightEyeCurrent();
  }

  drawTarget();

  gl::popMatrices();

  camera_.unsetCameras();


}

CINDER_APP_NATIVE( vizApp, RendererGl )
