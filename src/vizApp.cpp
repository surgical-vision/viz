#include "../include/vizApp.hpp"

using namespace viz;

const int WINDOW_WIDTH = 736;
const int WINDOW_HEIGHT = 288;

void vizApp::setup(){

  const std::string root_dir("../config");
  if (!boost::filesystem::is_directory(root_dir))
    throw std::runtime_error("Error, cannot file config dir!");

  const std::string input_video(root_dir + "/" + "f.avi");
  const std::string output_video(root_dir + "/" + "g.avi");
  const std::string camera_suj_file(root_dir + "/" +"cam_suj.txt");
  const std::string camera_j_file(root_dir + "/" +"cam_j.txt");
  const std::string left_dv_suj_file(root_dir + "/" +"left_dv_suj.txt");
  const std::string left_dv_j_file(root_dir + "/" +"left_dv_j.txt");
  const std::string right_dv_suj_file(root_dir + "/" +"right_dv_suj.txt");
  const std::string right_dv_j_file(root_dir + "/" +"right_dv_j.txt");
  const std::string da_vinci_config_file(root_dir + "/" +"da_vinci_config.json");

  handler_.reset(new ttrk::VideoHandler(input_video,output_video));
  camera_pose_.reset(new DaVinciPoseGrabber(camera_suj_file, camera_j_file));
  moving_objects_.push_back(boost::scoped_ptr<Trackable>(new Trackable()));
  moving_objects_.back()->setupDaVinciPoseGrabber(left_dv_suj_file, left_dv_j_file, da_vinci_config_file);
  moving_objects_.push_back(boost::scoped_ptr<Trackable>(new Trackable()));
  moving_objects_.back()->setupDaVinciPoseGrabber(right_dv_suj_file, right_dv_j_file, da_vinci_config_file);

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
