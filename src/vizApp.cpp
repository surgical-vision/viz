#include <CinderOpenCV.h>
#include <locale>

#include "../include/config_reader.hpp"
#include "../include/vizApp.hpp"
#include "../include/resources.hpp"

using namespace viz;

const int WINDOW_WIDTH = 736;
const int WINDOW_HEIGHT = 288;

//fairly good values for PSM 1 => Offset = [ -0.184, 0.04, 0.012, 0.008, 0.424, -0.496, -0.004, ]
// or Offset = [ -0.184, 0.024, 0.012, 0.052, 0.292, 0, 0, ]

void vizApp::setup(){

  std::vector<std::string> cmd_line_args = getArgs();

  if (cmd_line_args.size() < 2){
    throw std::runtime_error("Error.");
  }

  ConfigReader reader(cmd_line_args[1]);

  if (!boost::filesystem::is_directory(reader.get_element("root_dir")))
    throw std::runtime_error("Error, cannot file config dir!");

  if (!boost::filesystem::is_directory(reader.get_element("output"))){
    boost::filesystem::create_directory(reader.get_element("output"));
  }

  shader_ = gl::GlslProg(loadResource(RES_SHADER_VERT), loadResource(RES_SHADER_FRAG));

  //open video writer
  write_left_.open(reader.get_element("output") + "/" + reader.get_element("left_output_video"), CV_FOURCC('D', 'I', 'B', ' '), 25, cv::Size(WINDOW_WIDTH, WINDOW_HEIGHT));
  write_right_.open(reader.get_element("output") + "/" + reader.get_element("right_output_video"), CV_FOURCC('D', 'I', 'B', ' '), 25, cv::Size(WINDOW_WIDTH, WINDOW_HEIGHT));
  if (!write_left_.isOpened() || !write_right_.isOpened()){
    throw std::runtime_error("Error, couldn't open video file!");
  }
  
  //open video reader
  handler_.reset(new ttrk::StereoVideoHandler(reader.get_element("root_dir") + "/" + reader.get_element("left_input_video"), reader.get_element("root_dir") + "/" + reader.get_element("right_input_video"), ""));
  
  //open 
  ofs_cam_.open(reader.get_element("output") + "/" + reader.get_element("cam_se3_file"));
  if (!ofs_cam_.is_open()){
    throw std::runtime_error("Error, cannot open the camera processing file.");
  }

  camera_pg_.reset(new DaVinciPoseGrabber(reader.get_element("root_dir") + "/" + reader.get_element("camera_suj_file"), reader.get_element("root_dir") + "/" + reader.get_element("camera_j_file"), davinci::ECM));
  camera_.Setup(reader.get_element("root_dir") + "/" + reader.get_element("camera_config"), WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1000);

  camera_estimates_.reset(new PoseGrabber(reader.get_element("root_dir") + "/" + reader.get_element("camera_estimate_file")));
  camera_estimate_matrix_.setToIdentity();

  moving_objects_pg_.push_back(boost::shared_ptr<Trackable>(new Trackable()));
  moving_objects_pg_.back()->setupDaVinciPoseGrabber(reader.get_element("root_dir") + "/" + reader.get_element("psm2_suj_file"), reader.get_element("root_dir") + "/" + reader.get_element("psm2_j_file"), reader.get_element("output") + "/" + reader.get_element("psm1_dh_file"), reader.get_element("output") + "/" + reader.get_element("psm1_se3_file"), reader.get_element("root_dir") + "/" + reader.get_element("da_vinci_config_file"), davinci::PSM1);

  moving_objects_pg_.back()->getDVPoseGrabber()->setupOffsets(7);

  moving_objects_pg_.push_back(boost::shared_ptr<Trackable>(new Trackable()));
  moving_objects_pg_.back()->setupDaVinciPoseGrabber(reader.get_element("root_dir") + "/" + reader.get_element("psm1_suj_file"), reader.get_element("root_dir") + "/" + reader.get_element("psm1_j_file"), reader.get_element("output") + "/" + reader.get_element("psm1_dh_file"), reader.get_element("output") + "/" + reader.get_element("psm1_se3_file"), reader.get_element("root_dir") + "/" + reader.get_element("da_vinci_config_file"), davinci::PSM1);
  moving_objects_pg_.back()->getDVPoseGrabber()->setupOffsets(7);

  setWindowSize(2 * WINDOW_WIDTH, WINDOW_HEIGHT);
  framebuffer_ = gl::Fbo(WINDOW_WIDTH, WINDOW_HEIGHT);
  load_next_image_ = true;
  save_next_image_ = false;

  save_toggle_ = false;

  draw2 = true;
  draw3 = true;
  draw3d = false;

  // set up the camera
  CameraPersp cam;
  cam.setEyePoint(Vec3f(5.0f, 10.0f, 10.0f));
  cam.setCenterOfInterestPoint(Vec3f(0.0f, 2.5f, 0.0f));
  cam.setPerspective(60.0f, getWindowAspectRatio(), 1.0f, 1000.0f);
  maya_cam_.setCurrentCam(cam);

}

void vizApp::update(){

  camera_pose_ = camera_pg_->getPose(load_next_image_).poses_[0].first;

  camera_estimate_matrix_ = camera_estimates_->getPose(load_next_image_).poses_[0].first;

  for (std::size_t i = 0; i < moving_objects_pg_.size(); ++i){
    auto &mop = moving_objects_pg_[i];
    mop->getDaVinciPose(load_next_image_);
  }
  
  if (load_next_image_){

    cv::Mat stereo_image = handler_->GetNewFrame();
    
    //cv::Mat stereo_image = cv::Mat::zeros(cv::Size(WINDOW_WIDTH * 2, WINDOW_HEIGHT), CV_8UC3);
    //for (int r = 0; r < stereo_image.rows; ++r){
    //    for (int c = 0; c < stereo_image.cols; ++c){
    //        stereo_image.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 255);
    //    }
    //}
    

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

  if (draw3d){
    draw3D();
  }else{
    drawEye(right_texture_, false);
  }
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf((float)framebuffer_.getWidth(), (float)framebuffer_.getHeight(), 2.0 * framebuffer_.getWidth(), 0.0));
  saveFrame(framebuffer_.getTexture(), false); //remember only saves when save_next_frame_ is true, right call turns this off
  

}

void vizApp::draw3D(){
  
  gl::clear(Color(1, 0, 0));

  static ci::Matrix44f first_camera_pose = camera_pose_;

  ci::CameraPersp cam;
  cam.setEyePoint(ci::Vec3f(0, 0, 0));
  cam.setViewDirection(ci::Vec3f(0, 0, 1));
  cam.setWorldUp(ci::Vec3f(0, -1, 0));

  ci::CameraPersp maya;
  maya.setEyePoint(ci::Vec3f(100, 0, -150));
  maya.setWorldUp(ci::Vec3f(0, -1, 0));
  maya.lookAt(ci::Vec3f(0, 0, 5));
  
  
  gl::pushMatrices();

  //gl::setMatrices(maya_cam_.getCamera());
  gl::setMatrices(maya);

  camera_.setupCameras();

  gl::pushModelView();

  //ci::Matrix44f update_to_camera_pose = camera_pose_ * first_camera_pose.inverted();
  ci::Matrix44f update_to_camera_pose = first_camera_pose.inverted() * camera_pose_;

  gl::multModelView(update_to_camera_pose); 
  
  gl::drawFrustum(cam);
  
  gl::popModelView();

  gl::pushModelView(); 

  gl::multModelView(first_camera_pose.inverted());
  drawTarget();

  gl::popModelView();

  camera_.unsetCameras();
  gl::popMatrices();
  
  //draw camera frustrum
  //move camera to new viewpoint and point at origin
}

void vizApp::saveFrame(gl::Texture texture, bool isLeft){
  
  if (!save_next_image_){
    return;
  }

  cv::Mat frame = toOcv(texture), flipped;
  cv::flip(frame, flipped, 0);

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

      //if (i == 0)
      // ci::app::console() << "model view = " << ci::gl::getModelView() << std::endl;

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
    //ci::app::console() << "left cam ";
  }
  else{
    camera_.makeRightEyeCurrent();
    //ci::app::console() << "right cam ";
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
  
  ci::Matrix44d in_cam_coords = cam_pose.inverted() * obj_pose;

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
  else if (event.getChar() == 'j'){
    draw2 = !draw2;
    return;
  }
  else if (event.getChar() == 'k'){
    draw3 = !draw3;
    return;
  }
  else if (event.getChar() == 's'){
    save_toggle_ = !save_toggle_;
    return;
  }
  else if (event.getChar() == 'a'){
    draw3d = !draw3d;
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
  boost::shared_ptr< std::vector< double > > base_offsets = moving_objects_pg_[current_model_idx]->getDVPoseGrabber()->getBaseOffsets();

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

  if (event.getChar() == 'Q'){
      (*base_offsets)[0] += 0.001;
  }
  else if (event.getChar() == 'W'){
      (*base_offsets)[1] += 0.001;
  }
  else if (event.getChar() == 'E'){
      (*base_offsets)[2] += 0.001;
  }
  else if (event.getChar() == 'R'){
      (*base_offsets)[3] += 0.001;
  }
  else if (event.getChar() == 'T'){
      (*base_offsets)[4] += 0.001;
  }
  else if (event.getChar() == 'Y'){
      (*base_offsets)[5] += 0.001;
  }
  
  else if (event.getChar() == 'Z'){
      (*base_offsets)[0] -= 0.001;
  }
  else if (event.getChar() == 'X'){
      (*base_offsets)[1] -= 0.001;
  }
  else if (event.getChar() == 'C'){
      (*base_offsets)[2] -= 0.001;
  }
  else if (event.getChar() == 'V'){
      (*base_offsets)[3] -= 0.001;
  }
  else if (event.getChar() == 'B'){
      (*base_offsets)[4] -= 0.001;
  }
  else if (event.getChar() == 'N'){
      (*base_offsets)[5] -= 0.001;
  }
  


  if (offsets != nullptr){
    ci::app::console() << "Offset = [ ";
    for (auto i : *offsets){
      ci::app::console() << i << ", ";
    }
    ci::app::console() << "]" << std::endl;

    ci::app::console() << "Base Offset = [ ";
    for (auto i : *base_offsets){
        ci::app::console() << i << ", ";
    }
    ci::app::console() << "]" << std::endl;
  }

}

CINDER_APP_NATIVE( vizApp, RendererGl )
