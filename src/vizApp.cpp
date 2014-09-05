#include "../include/vizApp.hpp"
#include "../include/resources.hpp"
#include <CinderOpenCV.h>
#include <locale>

using namespace viz;

const int WINDOW_WIDTH = 736;
const int WINDOW_HEIGHT = 288;

//fairly good values for PSM 1 => Offset = [ -0.184, 0.04, 0.012, 0.008, 0.424, -0.496, -0.004, ]
// or Offset = [ -0.184, 0.024, 0.012, 0.052, 0.292, 0, 0, ]

void vizApp::setup(){

  const std::string root_dir("../config/synth/");
  if (!boost::filesystem::is_directory(root_dir))
    throw std::runtime_error("Error, cannot file config dir!");

  if (!boost::filesystem::is_directory(root_dir + "/output/")){
    boost::filesystem::create_directory(root_dir + "/output/");
  }

  shader_ = gl::GlslProg(loadResource(RES_SHADER_VERT), loadResource(RES_SHADER_FRAG));

  const std::string left_input_video(root_dir + "/" + "left.avi");
  const std::string right_input_video(root_dir + "/" + "right.avi");
  const std::string output_video(root_dir + "/" + "g.avi");

  const std::string camera_suj_file(root_dir + "/dv/" +"cam_suj.txt");
  const std::string camera_j_file(root_dir + "/dv/" + "cam_j.txt");
  const std::string cam_se3_file(root_dir + "/output/" + "cam_se3.txt");

  const std::string psm1_dv_suj_file(root_dir + "/dv/" +"psm1_suj.txt");
  const std::string psm1_dv_j_file(root_dir + "/dv/" + "psm1_j.txt");
  const std::string psm1_dh_file(root_dir + "/output/" + "psm1_dh.txt");
  const std::string psm1_se3_file(root_dir + "/output/" + "psm1_se3.txt");

  const std::string psm2_dv_suj_file(root_dir + "/dv/" + "psm2_suj.txt");
  const std::string psm2_dv_j_file(root_dir + "/dv/" + "psm2_j.txt");
  const std::string psm2_dh_file(root_dir + "/output/" + "psm2_dh.txt");
  const std::string psm2_se3_file(root_dir + "/output/" + "psm2_se3.txt");

  const std::string da_vinci_config_file(root_dir + "/dv/" +"model/model.json");

  write_left_.open(root_dir + "/output/ouput_left.avi", CV_FOURCC('D', 'I', 'B', ' '), 25, cv::Size(WINDOW_WIDTH, WINDOW_HEIGHT));
  write_right_.open(root_dir + "/output/ouput_right.avi", CV_FOURCC('D', 'I', 'B', ' '), 25, cv::Size(WINDOW_WIDTH, WINDOW_HEIGHT));

  if (!write_left_.isOpened() || !write_right_.isOpened()){
    throw std::runtime_error("Error, couldn't open video file!");
  }

  //handler_.reset(new ttrk::StereoVideoHandler(left_input_video, right_input_video, output_video));
  
  ofs_cam_.open(cam_se3_file);
  if (!ofs_cam_.is_open()){
    throw std::runtime_error("Error, cannot open the camera processing file.");
  }

  camera_pg_.reset(new DaVinciPoseGrabber(camera_suj_file, camera_j_file, davinci::ECM));
  camera_.Setup(root_dir + "/camera_config.xml", WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1000);
  
  moving_objects_pg_.push_back(boost::shared_ptr<Trackable>(new Trackable()));
  moving_objects_pg_.back()->setupDaVinciPoseGrabber(psm2_dv_suj_file, psm2_dv_j_file, psm2_dh_file, psm2_se3_file, da_vinci_config_file, davinci::PSM2);
  moving_objects_pg_.back()->getDVPoseGrabber()->setupOffsets(7);

  moving_objects_pg_.push_back(boost::shared_ptr<Trackable>(new Trackable()));
  moving_objects_pg_.back()->setupDaVinciPoseGrabber(psm1_dv_suj_file, psm1_dv_j_file, psm1_dh_file, psm1_se3_file, da_vinci_config_file, davinci::PSM1);
  moving_objects_pg_.back()->getDVPoseGrabber()->setupOffsets(7);

  //moving_objects_pg_.back().get<0>()->setupPoseGrabber(root_dir + "/dv/grid.txt");

  setWindowSize(2 * WINDOW_WIDTH, WINDOW_HEIGHT);
  framebuffer_ = gl::Fbo(WINDOW_WIDTH, WINDOW_HEIGHT);
  load_next_image_ = true;
  save_next_image_ = false;

  save_toggle_ = false;

  draw2 = true;
  draw3 = true;

}

void vizApp::update(){

  camera_pose_ = camera_pg_->getPose(load_next_image_).poses_[0].first;

  for (std::size_t i = 0; i < moving_objects_pg_.size(); ++i){
    auto &mop = moving_objects_pg_[i];
    mop->getDaVinciPose(load_next_image_);
  }
  
  if (load_next_image_){

    //cv::Mat stereo_image = handler_->GetNewFrame();
    cv::Mat stereo_image = cv::Mat::zeros(cv::Size(WINDOW_WIDTH * 2, WINDOW_HEIGHT), CV_8UC3);
    for (int r = 0; r < stereo_image.rows; ++r){
      for (int c = 0; c < stereo_image.cols; ++c){
        stereo_image.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 255);
      }
    }

    cv::Mat left_frame = stereo_image(cv::Rect(0, 0, stereo_image.cols / 2, stereo_image.rows));
    cv::Mat right_frame = stereo_image(cv::Rect(stereo_image.cols / 2, 0, stereo_image.cols / 2, stereo_image.rows));

    left_texture_ = fromOcv(left_frame);
    right_texture_ = fromOcv(right_frame);
    load_next_image_ = false;
    if (save_toggle_)
      save_next_image_ = true;
  }

}

void vizApp::draw(){
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) ); 

  framebuffer_.bindFramebuffer();
  drawEye(left_texture_, true);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf(0.0, (float)framebuffer_.getHeight(), (float)framebuffer_.getWidth(), 0.0));
  saveFrame(framebuffer_.getTexture(), true); //remember only saves when save_next_frame_ is true, right call turns this off

  framebuffer_.bindFramebuffer();
  drawEye(right_texture_, false);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf((float)framebuffer_.getWidth(), (float)framebuffer_.getHeight(), 2.0 * framebuffer_.getWidth(), 0.0));
  saveFrame(framebuffer_.getTexture(), false); //remember only saves when save_next_frame_ is true, right call turns this off
  

}

void vizApp::saveFrame(gl::Texture texture, bool isLeft){
  
  if (!save_next_image_){
    return;
  }

  cv::Mat frame = toOcv(texture), flipped;
  cv::flip(frame, flipped, 1);

  if (isLeft){
    write_left_ << flipped;

    for (std::size_t i = 0; i < moving_objects_pg_.size(); ++i){
      auto &pg = moving_objects_pg_[i];
      savePoseAsSE3(pg->getSe3Stream(), camera_pose_, pg->getPose());
      savePoseAsSE3AndDH(pg->getDHStream(), camera_pose_, pg->getPose());
    }

    savePoseAsSE3(ofs_cam_, ci::Matrix44d(), camera_pose_);
    
  }
  else{
    write_right_ << flipped;
    save_next_image_ = false;
  }

}

void vizApp::shutdown(){

  for (std::size_t i = 0; i < moving_objects_pg_.size(); ++i){
    auto &pg = moving_objects_pg_[i];
    pg->getSe3Stream().close();
    pg->getDHStream().close();
  }

  ofs_cam_.close();
  write_left_.release();
  write_right_.release();
  AppNative::shutdown();

}

void vizApp::drawTarget(){

  gl::enableDepthRead();
  gl::enableDepthWrite();
  
  //glEnable(GL_LIGHTING);
  //glEnable(GL_LIGHT0);

  //GLfloat light_position[] = { 0, 0, 0, 30 };
  //glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  //glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.0f);
  //glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.0f);
  //glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.00015f);

  
  for (auto mop : moving_objects_pg_){

    const Pose pose = mop->getPose();

    auto meshes_textures_transforms = mop->getDaVinciTrackable()->GetRenderableMeshes();

    for (std::size_t i = 0; i < meshes_textures_transforms.size(); ++i){

      if ((!draw2 && i == 2) || (!draw3 && i == 3)){
        continue;
      }

      //shader_.bind();
      //shader_.uniform("NumEnabledLights", 1);        

      gl::pushModelView();
      const auto &mesh = meshes_textures_transforms[i].get<0>();
      const auto &texture = meshes_textures_transforms[i].get<1>();
      //const auto &transform = mesh_tex_trans.get<2>();
     
      gl::multModelView(pose.poses_[i].first); //multiply by the pose of this tracked object

      gl::draw(*mesh);

      gl::popModelView();
      //shader_.unbind();
    }  

  }


  //glDisable(GL_LIGHTING);
}

void vizApp::drawEye(gl::Texture &texture, bool is_left){

  gl::clear(Color(0, 0, 0));

  gl::disableDepthRead();

  gl::draw(texture);

  camera_.setupCameras(); //do viewport cache and set model view to ident

  gl::pushMatrices();

  ci::gl::Light &light = camera_.getLight();
  light.enable();

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

  drawTarget();
  drawGrid();

  light.disable();

  gl::popMatrices(); 

  camera_.unsetCameras(); //reset the viewport values

}

void vizApp::drawGrid(float size, float step){
 
  gl::pushModelView();

  static bool first = true;
  static ci::Matrix44d trans;
  if (first){
 
    trans.setToIdentity();
    trans.setRow(0, ci::Vec4d(0.2112, 0.8628, -0.4594, -30.04));
    trans.setRow(1, ci::Vec4d(0.9749, -0.1520, 0.1628, 969.53));
    trans.setRow(2, ci::Vec4d(0.0706, -0.4822, -0.8732, 720.28));
    first = false;

  }
  
  gl::multModelView(trans);

  gl::color(Colorf(0.2f, 0.2f, 0.2f));

  for (float i = 0; i <= size; i += step){
    gl::drawLine(ci::Vec3f(0.0f, i, 0.0f), ci::Vec3f(size, i, 0.0f));
    gl::drawLine(ci::Vec3f(i, 0.0f, 0.0f), ci::Vec3f(i, size, 0.0f));
  }

  gl::color(1.0, 0.0, 0.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(5, 0, 0));
  gl::color(0.0, 1.0, 0.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(0, 5, 0));
  gl::color(0.0, 0.0, 1.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(0, 0, 5));


  gl::color(1.0, 1.0, 1.0);

  gl::popModelView();
}

inline void writePose(std::ofstream &ofs, const std::string &title, const ci::Matrix44d &obj_pose, const ci::Matrix44d &cam_pose){
  
  ci::Matrix44d in_cam_coords = obj_pose * cam_pose.inverted(); 

  ofs << "#" + title + "\n";
  ofs << in_cam_coords;
  ofs << "\n\n";

}

void vizApp::savePoseAsSE3(std::ofstream &ofs, const ci::Matrix44d &camera_pose, const Pose &pose){

  if (pose.poses_.size() != 4 && pose.poses_.size() != 1){
    throw std::runtime_error("Error, there should be 4 poses to call this function");
  }

  if (pose.poses_.size() == 1){
    writePose(ofs, "World to camera transform", pose.poses_[0].first, camera_pose);
  }
  else{
    writePose(ofs, "Camera to body transform", pose.poses_[0].first, camera_pose);

    writePose(ofs, "Camera to wrist transform", pose.poses_[1].first, camera_pose);

    writePose(ofs, "Camera to clasper 1", pose.poses_[2].first, camera_pose);

    writePose(ofs, "Camera to clasper 2", pose.poses_[3].first, camera_pose);
  }

}

void vizApp::savePoseAsSE3AndDH(std::ofstream &ofs, const ci::Matrix44d &camera_pose, const Pose &pose){

  if (pose.poses_.size() != 4){
    throw std::runtime_error("Error, there should be 4 poses to call this function");
  }

  writePose(ofs, "Camera to body transform", pose.poses_[0].first, camera_pose);

  ofs << "# DH parameter from body to wrist \n";
  ofs << pose.poses_[1].second;
  ofs << "\n\n";

  ofs << "# DH parameter from wrist to clasper 1 \n";
  ofs << pose.poses_[2].second;
  ofs << "\n\n";

  ofs << "# DH parameter from wrist to clasper 2 \n";
  ofs << pose.poses_[3].second;
  ofs << "\n\n";
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
    return;
  }

  if (event.getChar() == 'j'){
    draw2 = !draw2;
    return;
  }

  if (event.getChar() == 'k'){
    draw3 = !draw3;
    return;
  }

  if (event.getChar() == 's'){
    save_toggle_ = !save_toggle_;
    return;
  }

  static int current_model_idx = 0;

  if (moving_objects_pg_.size() == 0)
    return;

  if (std::isdigit(event.getChar())){
    std::stringstream ss;
    ss << event.getChar();
    ss >> current_model_idx;
    return;
  }

  if (current_model_idx >= (int)moving_objects_pg_.size()){
    console() << "Error, this index is too large for the vector!" << std::endl;
  }
  
  boost::shared_ptr< std::vector< double > > offsets = moving_objects_pg_[current_model_idx]->getDVPoseGrabber()->getOffsets();

  if (offsets->size() == 0)
    return;

  if (event.getChar() == 'q'){
    (*offsets)[0] += 0.004;
  }
  else if (event.getChar() == 'w'){
    (*offsets)[1] += 0.004;
  }
  else if (event.getChar() == 'e'){
    (*offsets)[2] += 0.004;
  }
  else if (event.getChar() == 'r'){
    (*offsets)[3] += 0.004;
  }
  else if (event.getChar() == 't'){
    (*offsets)[4] += 0.004;
  }
  else if (event.getChar() == 'y'){
    (*offsets)[5] += 0.004;
  }
  else if (event.getChar() == 'u'){
    (*offsets)[6] += 0.004;
  }

  else if (event.getChar() == 'z'){
    (*offsets)[0] -= 0.004;
  }
  else if (event.getChar() == 'x'){
    (*offsets)[1] -= 0.004;
  }
  else if (event.getChar() == 'c'){
    (*offsets)[2] -= 0.004;
  }
  else if (event.getChar() == 'v'){
    (*offsets)[3] -= 0.004;
  }
  else if (event.getChar() == 'b'){
    (*offsets)[4] -= 0.004;
  }
  else if (event.getChar() == 'n'){
    (*offsets)[5] -= 0.004;
  }
  else if (event.getChar() == 'm'){
    (*offsets)[6] -= 0.004;
  }

  if (offsets != nullptr){
    ci::app::console() << "Offset = [ ";
    for (auto i : *offsets){
      ci::app::console() << i << ", ";
    }
    ci::app::console() << "]" << std::endl;
  }

}

CINDER_APP_NATIVE( vizApp, RendererGl )
