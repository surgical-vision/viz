#include "../include/vizApp.hpp"
#include <CinderOpenCV.h>

using namespace viz;

const int WINDOW_WIDTH = 736;
const int WINDOW_HEIGHT = 288;

void vizApp::setup(){

  const std::string root_dir("../config");
  if (!boost::filesystem::is_directory(root_dir))
    throw std::runtime_error("Error, cannot file config dir!");

  const std::string left_input_video(root_dir + "/" + "left.avi");
  const std::string right_input_video(root_dir + "/" + "right.avi");
  const std::string output_video(root_dir + "/" + "g.avi");
  const std::string camera_suj_file(root_dir + "/dv/" +"cam_suj.txt");
  const std::string camera_j_file(root_dir + "/dv/" + "cam_j.txt");
  const std::string psm1_suj_file(root_dir + "/dv/" +"psm1_suj.txt");
  const std::string psm1_j_file(root_dir + "/dv/" +"psm1_j.txt");
  const std::string psm2_suj_file(root_dir + "/dv/" + "psm2_suj.txt");
  const std::string psm2_dv_j_file(root_dir + "/dv/" + "psm2_j.txt");
  const std::string da_vinci_config_file(root_dir + "/dv/" +"model/model.json");

  CameraPersp cam;
  cam.setEyePoint(ci::Vec3f(0, 0, 0));
  cam.setViewDirection(ci::Vec3f(0, 0, -1));
  cam.setWorldUp(ci::Vec3f(0, 1, 0));
  maya_cam_.setCurrentCam(cam);

  handler_.reset(new ttrk::StereoVideoHandler(left_input_video, right_input_video, output_video));
  //tmp_handler_.reset(new ttrk::ImageHandler(root_dir + "/right/", root_dir + "/right_res/"));
  
  camera_pg_.reset(new DaVinciPoseGrabber(camera_suj_file, camera_j_file, davinci::ECM));
  camera_.Setup(root_dir + "/camera_config.xml", WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1000);
    
  moving_objects_pg_.push_back( std::make_pair<>(boost::shared_ptr<Trackable>(new Trackable()),ci::Matrix44f()));
  moving_objects_pg_.back().first->setupDaVinciPoseGrabber(psm2_suj_file, psm2_dv_j_file, da_vinci_config_file, davinci::PSM2);
  //moving_objects_pg_.push_back(std::make_pair<>(boost::scoped_ptr<Trackable>(new Trackable()), ci::Matrix44f()));
  //moving_objects_pg_.back().first->setupDaVinciPoseGrabber(psm1_suj_file, psm1_dv_j_file, da_vinci_config_file, davinci::PSM1);
    
  setWindowSize(2 * WINDOW_WIDTH, WINDOW_HEIGHT);
  framebuffer_ = gl::Fbo(WINDOW_WIDTH, WINDOW_HEIGHT);
  load_next_image_ = true;

  srand(time(NULL));

}

void vizApp::update(){

  if (load_next_image_){

    cv::Mat stereo_image = handler_->GetNewFrame();

    cv::Mat left_frame = stereo_image(cv::Rect(0, 0, stereo_image.cols / 2, stereo_image.rows));
    cv::Mat right_frame = stereo_image(cv::Rect(stereo_image.cols/2, 0, stereo_image.cols / 2, stereo_image.rows));

    left_texture_ = fromOcv(left_frame);
    right_texture_ = fromOcv(right_frame);

    load_next_image_ = false;

    camera_pose_ = camera_pg_->getNextPose().poses_[0];

    ci::app::console() << "Camera Pose = \n\n" << camera_pose_ << "\n";

    for (std::size_t i = 0; i < moving_objects_pg_.size(); ++i){
      auto &mop = moving_objects_pg_[i];

      mop.second = mop.first->getDVPoseGrabber()->getNextPose().poses_[0];

      ci::app::console() << "Tool Pose = \n\n" << mop.second << "\n";
    }

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

  for (auto mop : moving_objects_pg_){

    gl::pushModelView();

    const auto &pose = mop.second;
    gl::multModelView(pose); //multiply by the pose of this tracked object

    auto meshes_textures_transforms = mop.first->getDaVinciTrackable()->GetRenderableMeshes();

    for (auto mesh_tex_trans : meshes_textures_transforms){

      const auto &mesh = mesh_tex_trans.get<0>();
      const auto &texture = mesh_tex_trans.get<1>();
      const auto &transform = mesh_tex_trans.get<2>();

      gl::pushModelView();

      gl::multModelView(transform);
      gl::draw(*mesh);
      
      gl::popModelView();
    }

    gl::popModelView();

  }



}


void vizApp::drawEye(gl::Texture &texture, bool is_left){

  gl::clear(Color(0, 0, 0));

  gl::disableDepthRead();

  gl::draw(texture);

  camera_.setupCameras(); //do viewport cache and set model view to ident

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

  CameraPersp p = maya_cam_.getCamera();
  ci::Vec3f tip_in_eye = p.worldToEye(moving_objects_pg_[0].second.getTranslate().xyz());
 
  ci::app::console() << "Point in eye = \n " << tip_in_eye << std::endl;

  drawTarget();
  drawGrid();

  gl::popMatrices(); 

  camera_.unsetCameras(); //reset the viewport values

}


void vizApp::drawGrid(float size, float step){
  
  //gl::pushModelView();

  //static bool first = true;
  //static std::vector<ci::Vec3f> points;
  //if (first){

  //  srand(time(NULL));
  //  
  //  for (int i = 0; i < 300; ++i){

  //    float x = (float)rand() / RAND_MAX;
  //    float y = (float)rand() / RAND_MAX;
  //    float z = (float)rand() / RAND_MAX;

  //    x = (x * 1000) - 500;
  //    y = (y * 1000) - 500;
  //    z = (z * 1000) - 500;

  //    points.push_back(ci::Vec3f(x, y, z));

  //  }

  //  first = false;
  //}

  //for (auto pt : points){
  //  gl::drawSphere(pt, 4);
  //}

  //gl::popModelView();

  //return;

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


void vizApp::mouseDrag(MouseEvent event){
  // let the camera handle the interaction
  maya_cam_.mouseDrag(event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown());
}

void vizApp::mouseDown(MouseEvent event){
  maya_cam_.mouseDown(event.getPos());
}

void vizApp::keyDown(KeyEvent event){

  if (event.getChar() == ' '){
    load_next_image_ = true;
  }

}

CINDER_APP_NATIVE( vizApp, RendererGl )
