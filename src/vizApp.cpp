#include "../include/vizApp.hpp"
#include <CinderOpenCV.h>

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

  CameraPersp cam;
  cam.setEyePoint(ci::Vec3f(0, 0, 0));
  cam.setViewDirection(ci::Vec3f(0, 0, -1));
  cam.setWorldUp(ci::Vec3f(0, 1, 0));
  maya_cam_.setCurrentCam(cam);

  //handler_.reset(new ttrk::VideoHandler(input_video,output_video));
  handler_.reset(new ttrk::ImageHandler(root_dir + "/left/", root_dir + "/left_res/"));
  tmp_handler_.reset(new ttrk::ImageHandler(root_dir + "/right/", root_dir + "/right_res/"));
  camera_pg_.reset(new PoseGrabber(root_dir + "/camera_poses.txt"));
  camera_.Setup(root_dir + "/camera_config.xml", WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1000);
  
  /*
  camera_.reset(new DaVinciPoseGrabber(camera_suj_file, camera_j_file));
  moving_objects_.push_back(boost::scoped_ptr<Trackable>(new Trackable()));
  moving_objects_.back()->setupDaVinciPoseGrabber(left_dv_suj_file, left_dv_j_file, da_vinci_config_file);
  moving_objects_.push_back(boost::scoped_ptr<Trackable>(new Trackable()));
  moving_objects_.back()->setupDaVinciPoseGrabber(right_dv_suj_file, right_dv_j_file, da_vinci_config_file);
  */
  
  setWindowSize(2 * WINDOW_WIDTH, WINDOW_HEIGHT);
  framebuffer_ = gl::Fbo(WINDOW_WIDTH, WINDOW_HEIGHT);
  load_next_image_ = true;

}

void vizApp::update(){

  if (load_next_image_){

    cv::Mat l_image = handler_->GetNewFrame();
    left_texture_ = fromOcv(l_image);
    cv::Mat r_image = tmp_handler_->GetNewFrame();
    right_texture_ = fromOcv(r_image);

    load_next_image_ = false;

    camera_pose_ = camera_pg_->getNextPose().poses_[0];

  }

}

void vizApp::draw(){
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) ); 

  framebuffer_.bindFramebuffer();
  drawEye(left_texture_, true);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf(0.0, (float)framebuffer_.getHeight(), (float)framebuffer_.getWidth(), 0.0));

  framebuffer_.bindFramebuffer();
  drawEye(right_texture_, false);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf((float)framebuffer_.getWidth(), (float)framebuffer_.getHeight(), 2.0 * framebuffer_.getWidth(), 0.0));

}

void vizApp::drawTarget(){
  drawGrid();
}


void vizApp::drawEye(gl::Texture &texture, bool is_left){

  gl::clear(Color(0, 0, 0));

  gl::disableDepthRead();

  gl::draw(texture);

  camera_.setupCameras(); //do viewport cache

  gl::pushMatrices();

  if (is_left){
    camera_.moveEyeToLeftCam(maya_cam_, camera_pose_); //set the position/modelview of the camera (setViewDirection etc)
  }
  else{
    camera_.moveEyeToRightCam(maya_cam_, camera_pose_);
  } 

  if (is_left){
    camera_.makeLeftEyeCurrent();
  }
  else{
    camera_.makeRightEyeCurrent();

  }

  //gl::multModelView(camera_pose_);
  
  drawTarget();

  gl::popMatrices(); 

  camera_.unsetCameras(); //reset the viewport values

}


void vizApp::mouseDrag(MouseEvent event){
  // let the camera handle the interaction
  //maya_cam_.mouseDrag(event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown());
}

void vizApp::mouseDown(MouseEvent event){
  //maya_cam_.mouseDown(event.getPos());
}

void vizApp::keyDown(KeyEvent event){

  if (event.getChar() == ' '){
    load_next_image_ = true;
  }

}

void vizApp::drawGrid(float size, float step){
  
  gl::color(Colorf(0.2f, 0.2f, 0.2f));

  for (float i = 0; i <= size; i += step){
    gl::drawLine(ci::Vec3f(0.0f, i, 0.0f), ci::Vec3f(size, i, 0.0f));
    gl::drawLine(ci::Vec3f(i, 0.0f, 0.0f), ci::Vec3f(i, size, 0.0f));
  }

  gl::color(1.0, 0.0, 0.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(30, 0, 0));
  gl::color(0.0, 1.0, 0.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(0, 30, 0));
  gl::color(0.0, 0.0, 1.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(0, 0, 30));


  gl::color(1.0, 1.0, 1.0);
}


CINDER_APP_NATIVE( vizApp, RendererGl )
