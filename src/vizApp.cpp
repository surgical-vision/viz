#include <CinderOpenCV.h>
#include <locale>
#include <opencv2/highgui.hpp>

#include "../include/config_reader.hpp"
#include "../include/vizApp.hpp"
#include "../include/resources.hpp"

using namespace viz;

void vizApp::setup(){

  std::vector<std::string> cmd_line_args = getArgs();

  if (cmd_line_args.size() < 2){
    throw std::runtime_error("Error.");
  }

  ConfigReader reader(cmd_line_args[1]);

  if (!boost::filesystem::is_directory(reader.get_element("root-dir")))
    throw std::runtime_error("Error, cannot file config dir!");

  if (!boost::filesystem::is_directory(reader.get_element("output"))){
    boost::filesystem::create_directory(reader.get_element("output"));
  }

  video_left_ = VideoIO(reader.get_element("left-input-video"), reader.get_element("output-dir") + "/" + reader.get_element("left-output-video"));  
  video_right_ = VideoIO(reader.get_element("right-input-video"), reader.get_element("output-dir") + "/" + reader.get_element("right-output-video"));

  camera_.Setup(reader.get_element("root_dir") + "/" + reader.get_element("camera_config"), reader.get_element_as_int("window_width"), reader.get_element_as_int("window_height"), 1, 1000);

  try{
    moveable_camera_.reset(new PoseGrabber(reader));
    tracked_camera_.reset(new PoseGrabber(reader)); //if there is no moveable camera then there won't be a tracked camera
  }
  catch (std::runtime_error){

  }

  camera_image_width_ = reader.get_element_as_int("window-width");
  camera_image_height_ = reader.get_element_as_int("window-height");

  framebuffer_ = gl::Fbo(camera_image_width_, camera_image_height_);
  framebuffer_3d_ = gl::Fbo(three_dim_viz_width_, three_dim_viz_height_);

  setWindowSize((2 * framebuffer_.getWidth()), framebuffer_.getHeight() + framebuffer_3d_.getHeight());

  run_video_ = false;
  load_next_image_ = true;
  save_toggle_ = false;

}

void vizApp::update(){

 
  /**
  * update the trackable object and camera poses.
  * update_toggle_ is used when manipulating the object poses in the UI (to remove constant offsets).
  * this sets internal offset parameters inside the objects so their pose needs to be 'refreshed' to get the offset values.
  */
  if (load_next_image_ || run_video_ || update_toggle_){

    if (moveable_camera_) moveable_camera_->LoadPose(update_toggle_);
    if (tracked_camera_) tracked_camera_->LoadPose(update_toggle_);
    for (size_t i = 0; i < trackables_.size(); ++i){
      trackables_[i]->LoadPose(update_toggle_);
    }
  }

  //ci::Matrix44f flip;
  //flip.setToIdentity();
  //flip.at(1, 1) = -1;
  //flip.at(2, 2) = -1;

  //camera_estimate_matrix_ = first_camera_pose * flip * camera_estimate_matrix_ * flip;



  if (load_next_image_ || run_video_){
    cv::Mat stereo_image;

    cv::Mat left_frame = video_left_.Read();
    cv::Mat right_frame = video_right_.Read();

    left_texture_ = fromOcv(left_frame);
    right_texture_ = fromOcv(right_frame);
    load_next_image_ = false;
  }

}

void vizApp::draw2D(gl::Texture &tex){
  
  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);
  glViewport(0, 0, camera_image_width_, camera_image_height_);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  
  glLoadIdentity();
  //glOrtho(-1, 1, -1, 1, -1, 1);
  glOrtho(0, camera_image_width_, 0, camera_image_height_, 0, 1);
  
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  
  glDisable(GL_DEPTH_TEST);

  tex.setFlipped(true);

  gl::draw(tex);

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glViewport(vp[0], vp[1], vp[2], vp[3]);
}

void vizApp::draw(){
	// clear out the window with black
  gl::clear( Color( 0, 0, 0 ) ); 

  framebuffer_.bindFramebuffer();
  drawEye(left_texture_, true);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf(0.0, (float)framebuffer_.getHeight(), (float)framebuffer_.getWidth(), 0.0));
  saveFrame(framebuffer_.getTexture(), true); //remember only saves when save_next_frame_ is true, right call turns this off

  gl::Texture test_fb = framebuffer_.getTexture();
  cv::Mat i = toOcv(test_fb);
  gl::Texture copy = fromOcv(i);

  framebuffer_.bindFramebuffer();
  drawEye(right_texture_, false);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf((float)framebuffer_.getWidth(), (float)framebuffer_.getHeight(), 2.0 * framebuffer_.getWidth(), 0.0));
  saveFrame(framebuffer_.getTexture(), false); //remember only saves when save_next_frame_ is true, right call turns this off

  framebuffer_3d_.bindFramebuffer();
  draw3D(copy);
  framebuffer_3d_.unbindFramebuffer();
  gl::draw(framebuffer_3d_.getTexture(), ci::Rectf(0.0, (float)framebuffer_.getHeight() + (float)framebuffer_3d_.getHeight(), (float)framebuffer_3d_.getWidth(), (float)framebuffer_.getHeight())); 

  framebuffer_3d_.bindFramebuffer();
  drawCameraTracker();
  framebuffer_3d_.unbindFramebuffer();
  gl::draw(framebuffer_3d_.getTexture(), ci::Rectf((float)framebuffer_3d_.getWidth(), (float)framebuffer_.getHeight() + (float)framebuffer_3d_.getHeight(), (float)framebuffer_3d_.getWidth() * 2, (float)framebuffer_.getHeight()));

}

void vizApp::drawTrajectories(const std::vector<ci::Matrix44f> &transforms, ci::Color &color){

  gl::color(color);

  for (size_t i = 1; i < transforms.size(); ++i){
    gl::drawLine(transforms[i - 1].getTranslate().xyz(), transforms[i].getTranslate().xyz());
  }

  gl::pushModelView();

  gl::multModelView(transforms.back());
  
  drawCamera(gl::Texture());

  gl::popModelView();

  gl::color(1.0, 1.0, 1.0);

  return;

}

void vizApp::drawImageOnCamera(gl::Texture &image_data, ci::Vec3f &tl, ci::Vec3f &bl, ci::Vec3f &tr, ci::Vec3f &br){
  
  ci::gl::SaveTextureBindState saveBindState(image_data.getTarget());
  ci::gl::BoolState saveEnabledState(image_data.getTarget());
  ci::gl::ClientBoolState vertexArrayState(GL_VERTEX_ARRAY);
  ci::gl::ClientBoolState texCoordArrayState(GL_TEXTURE_COORD_ARRAY);
  image_data.enableAndBind();

  glEnableClientState(GL_VERTEX_ARRAY);
  GLfloat verts[12];
  glVertexPointer(3, GL_FLOAT, 0, verts);
  
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  GLfloat texCoords[8];
  glTexCoordPointer(2, GL_FLOAT, 0, texCoords);

  for (int i = 0; i < 3; ++i) { verts[0 * 3 + i] = tl[i]; }
  for (int i = 0; i < 3; ++i) { verts[1 * 3 + i] = bl[i]; }
  for (int i = 0; i < 3; ++i) { verts[2 * 3 + i] = tr[i]; }
  for (int i = 0; i < 3; ++i) { verts[3 * 3 + i] = br[i]; }

  texCoords[0 * 2 + 0] = 0; texCoords[0 * 2 + 1] = 0;
  texCoords[1 * 2 + 0] = 0; texCoords[1 * 2 + 1] = 1;
  texCoords[2 * 2 + 0] = 1; texCoords[2 * 2 + 1] = 0;
  texCoords[3 * 2 + 0] = 1; texCoords[3 * 2 + 1] = 1;

  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

void vizApp::drawCamera(gl::Texture &image_data){
  
  ci::Vec3f vertex[8];
  
  ci::Vec3f eye(0, 0, 0);
  ci::Vec3f bottom_left(-5, -5, 20);
  ci::Vec3f bottom_right(5, -5, 20);
  ci::Vec3f top_left(-5, 5, 20);
  ci::Vec3f top_right(5, 5, 20);
 
  if (image_data)
    drawImageOnCamera(image_data, top_left, bottom_left, top_right, bottom_right);

  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, 0, &vertex[0].x);

  vertex[0] = eye;
  vertex[1] = bottom_left;
  vertex[2] = eye;
  vertex[3] = bottom_right;
  vertex[4] = eye;
  vertex[5] = top_left;
  vertex[6] = eye;
  vertex[7] = top_right;
  glDrawArrays(GL_LINES, 0, 8);

  glLineWidth(2.0f);
  vertex[0] = bottom_left;
  vertex[1] = bottom_right;
  vertex[2] = top_right;
  vertex[3] = top_left;
  glDrawArrays(GL_LINE_LOOP, 0, 4);

  glLineWidth(1.0f);
  glDisableClientState(GL_VERTEX_ARRAY);

}

void vizApp::drawCameraTracker(){

  if (!moveable_camera_ || !tracked_camera_) return;
  if (moveable_camera_->History().size() == 0 || tracked_camera_->History().size() == 0) return;

  gl::clear(Color(0.0, 0.0, 0.0));

  //set up a camera looking at the 'real' camera origin.
  ci::CameraPersp maya;
  maya.setEyePoint(moveable_camera_->History().back().getTranslate().xyz() + ci::Vec3f(30, 60, 60));
  maya.setWorldUp(ci::Vec3f(0, -1, 0));
  maya.lookAt(moveable_camera_->History().back().getTranslate().xyz());

  gl::pushMatrices();
  gl::setMatrices(maya);
  
  ci::Area viewport = gl::getViewport();
  gl::setViewport(ci::Area(0, 0, framebuffer_.getWidth(), framebuffer_.getHeight()));
  
  drawTrajectories(moveable_camera_->History(), ci::Color(0.0, 1.0, 1.0));

  drawTrajectories(tracked_camera_->History(), ci::Color(0.4, 1.0, 0.2));

  gl::setViewport(viewport);

  gl::popMatrices();

}

void vizApp::draw3D(gl::Texture &image){

  gl::clear(Color(0.0, 0.0, 0.0));
  
  //

  //ci::CameraPersp maya;
  //maya.setEyePoint(first_camera_pose.getTranslate().xyz() + ci::Vec3f(30, 100, 190));
  //maya.setWorldUp(ci::Vec3f(0, -1, 0));
  //maya.lookAt(first_camera_pose.getTranslate().xyz());// ci::Vec3f(0, 0, 5));
  //

  //gl::pushMatrices();
  //gl::setMatrices(maya);

  //drawGrid(5000, 1, first_camera_pose.getTranslate().xyz()[2] - 200);

  //ci::Area viewport = gl::getViewport();
  //gl::setViewport(ci::Area(0, 0, framebuffer_.getWidth(), framebuffer_.getHeight()));

  //gl::pushModelView();
  //gl::multModelView(camera_pose_);

  //drawCamera(image);
  //gl::popModelView();

  /////////////////

  //gl::pushModelView();

  //gl::multModelView(first_camera_pose * camera_estimate_matrix_);
  ////drawCamera();
  //gl::popModelView();

  //gl::pushModelView(); 

  ////gl::multModelView(first_camera_pose.inverted());
  //drawTarget();

  //gl::popModelView();

  //gl::setViewport(viewport);
  //gl::popMatrices();
  //
  //draw camera frustrum
  //move camera to new viewpoint and point at origin
}

void vizApp::saveFrame(gl::Texture texture, bool isLeft){
  
 /* if (!save_next_image_ && !manual_control_toggle_){
    return;
  }

  cv::Mat frame = toOcv(texture), flipped;
  cv::flip(frame, flipped, 0);
*/
 /* if (isLeft){
    write_left_ << flipped;

    for (std::size_t i = 0; i < moving_objects_pg_.size(); ++i){
      auto &pg = moving_objects_pg_[i];
      savePoseAsSE3(pg->getSe3FromCamStream(), camera_pg_->getPose(), pg->getPose());
      savePoseAsSE3(pg->getSe3FromWorldStream(), Pose(ci::Matrix44f()), pg->getPose());
      savePoseAsSE3AndDH(pg->getSe3AndDHStream(), camera_pg_->getPose(), pg->getPose());
      saveDH(pg->getSujDHStream(), pg->getJDHStream(), pg->getPose());
    }

    savePoseAsSE3(camera_pg_->getSe3FromWorldStream(), Pose(ci::Matrix44f()), camera_pg_->getPose());
    saveDH(camera_pg_->getSujDHStream(), camera_pg_->getJDHStream(), camera_pg_->getPose());

  }
  else{
    write_right_ << flipped;
    save_next_image_ = false;
  }*/

}

void vizApp::shutdown(){

  //for (std::size_t i = 0; i < moving_objects_pg_.size(); ++i){
  //  auto &pg = moving_objects_pg_[i];
  //  pg->getSe3FromWorldStream().close();
  //  pg->getSe3FromCamStream().close();
  //  pg->getSe3AndDHStream().close();
  //  pg->getSujDHStream().close();
  //  pg->getJDHStream().close();
  //}


  
  AppNative::shutdown();

}

void vizApp::drawTarget(){

  gl::enableDepthRead();
  gl::enableDepthWrite();
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

 // GLfloat light_position[] = { 0, 0, 0, 30 };
 ///glLightfv(GL_LIGHT0, GL_POSITION, light_position);
 // //glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.0f);
 // //glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.0f);
 // //glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.00015f);

  
  //for (auto mop : moving_objects_pg_){

  //  const Pose pose = mop->getPose();
  //  
  //  auto meshes_textures_transforms = mop->getDaVinciTrackable()->GetRenderableMeshes();

  //  for (std::size_t i = 0; i < meshes_textures_transforms.size(); ++i){

  //    if ((!draw2 && i == 2) || (!draw3 && i == 3)){
  //      continue;
  //    }

  //    //shader_.bind();
  //    //shader_.uniform("NumEnabledLights", 1);        

  //    gl::pushModelView();
  //    const auto &mesh = meshes_textures_transforms[i].get<0>();
  //    const auto &texture = meshes_textures_transforms[i].get<1>();

  //    gl::multModelView(pose.poses_[i].first); //multiply by the pose of this tracked object
  //    ci::app::console() << "Model view = \n" << gl::getModelView() << std::endl;
  //    //gl::multModelView(transform);

  //    gl::draw(*mesh);

  //    //shader_.unbind();
  //    gl::popModelView();

  //  }  

  //}


  glDisable(GL_LIGHTING);
}

ci::Matrix44f vizApp::getCameraPose(){

  if (moveable_camera_){
    return moveable_camera_->GetPose();
  }
  else{
    return ci::Matrix44f(); //return identity
  }

}

void vizApp::drawEye(gl::Texture &texture, bool is_left){

  gl::clear(Color(0, 0, 0));

  gl::disableDepthRead();

  //gl::draw(texture);
  draw2D(texture);
  
  camera_.setupCameras(); //do viewport cache and set model view to ident

  gl::pushMatrices();

  ci::gl::Light &light = camera_.getLight();
  light.enable();

  if (is_left){
    camera_.moveEyeToLeftCam(maya_cam_, getCameraPose()); //set the position/modelview of the camera (setViewDirection etc)
  }
  else{
    camera_.moveEyeToRightCam(maya_cam_, getCameraPose());
  }

  if (is_left){
    camera_.makeLeftEyeCurrent();
  }
  else{
    camera_.makeRightEyeCurrent();
  }
  
  drawTarget();
  
  light.disable();

  gl::popMatrices(); 

  camera_.unsetCameras(); //reset the viewport values

}

void vizApp::loadTrackables(const ConfigReader &reader){

  for (int i = 0;; ++i){

    try{

      std::stringstream s("trackable-"); 
      s << i;
      loadTrackable(s.str());

    }
    catch (std::runtime_error){
      break;
    }

  }

}

void vizApp::loadTrackable(const std::string &filepath){

  ConfigReader reader(filepath);
  try{
    trackables_.push_back(boost::shared_ptr<BasePoseGrabber>(new DHDaVinciPoseGrabber(reader)));
  }
  catch (std::runtime_error){

  }
  try{
    trackables_.push_back(boost::shared_ptr<BasePoseGrabber>(new SE3DaVinciPoseGrabber(reader)));
  }
  catch (std::runtime_error){

  }
  try{
    trackables_.push_back(boost::shared_ptr<BasePoseGrabber>(new PoseGrabber(reader)));
  }
  catch (std::runtime_error){

  }
}

void vizApp::drawGrid(float size, float step, float plane_position){
 
 // gl::pushModelView();
  
  gl::color(Colorf(0.5f, 0.5f, 0.5f));

  float start = -size;

  for (float i = -size; i <= size; i += step){
    //gl::drawLine(ci::Vec3f(0.0f, i, 0.0f), ci::Vec3f(size, i, 0.0f));
    //gl::drawLine(ci::Vec3f(i, 0.0f, 0.0f), ci::Vec3f(i, size, 0.0f));
    gl::drawLine(ci::Vec3f(start, i, plane_position), ci::Vec3f(size, i, plane_position));
    gl::drawLine(ci::Vec3f(i, start, plane_position), ci::Vec3f(i, size, plane_position));
  }

  gl::color(1.0, 1.0, 1.0);
  /*gl::color(1.0, 0.0, 0.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(5, 0, 0));
  gl::color(0.0, 1.0, 0.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(0, 5, 0));
  gl::color(0.0, 0.0, 1.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(0, 0, 5));*/


  //gl::color(1.0, 1.0, 1.0);

  //gl::popModelView();
}

inline void writePose(std::ofstream &ofs, const std::string &title, const ci::Matrix44d &obj_pose, const ci::Matrix44d &cam_pose){
  
  ci::Matrix44d in_cam_coords = cam_pose.inverted() * obj_pose;

  ofs << "#" + title + "\n";
  ofs << in_cam_coords;
  ofs << "\n\n";

}

//void vizApp::saveDH(std::ofstream &suj_ofs, std::ofstream &j_ofs, const Pose &pose){

  //for (auto &j_dh : pose.j_dh_vals_){
  //  j_ofs << j_dh << "\n";
  //}
  //j_ofs << "\n";

  //for (auto &suj_dh : pose.suj_dh_vals_){
  //  suj_ofs << suj_dh << "\n";
  //}
  //suj_ofs << "\n";

//}

//void vizApp::savePoseAsSE3(std::ofstream &ofs, const Pose &camera_pose, const Pose &pose){

  //if (pose.poses_.size() != 4 && pose.poses_.size() != 1){
  //  throw std::runtime_error("Error, there should be 4 poses to call this function");
  //}

  //if (pose.poses_.size() == 1){
  //  writePose(ofs, "World to camera transform", pose.poses_[0].first, camera_pose.poses_[0].first);
  //}
  //else{
  // 
  //  writePose(ofs, "Transform to body", pose.poses_[0].first, camera_pose.poses_[0].first);

  //  writePose(ofs, "Transform to wrist", pose.poses_[1].first, camera_pose.poses_[0].first);

  //  writePose(ofs, "Transform to clasper 1", pose.poses_[2].first, camera_pose.poses_[0].first);

  //  writePose(ofs, "Transform to clasper 2", pose.poses_[3].first, camera_pose.poses_[0].first);
  //}

//}

//void vizApp::savePoseAsSE3AndDH(std::ofstream &ofs, const Pose &camera_pose, const Pose &pose){

 /* if (pose.poses_.size() != 4){
    throw std::runtime_error("Error, there should be 4 poses to call this function");
  }

  writePose(ofs, "Camera to body transform", pose.poses_[0].first, camera_pose.poses_[0].first);

  writePose(ofs, "World to body transform", pose.poses_[0].first, ci::Matrix44f());
  
  ofs << "# DH parameter from body to wrist \n";
  ofs << pose.poses_[1].second;
  ofs << "\n\n";

  ofs << "# DH parameter from wrist to clasper 1 \n";
  ofs << pose.poses_[2].second;
  ofs << "\n\n";

  ofs << "# DH parameter from wrist to clasper 2 \n";
  ofs << pose.poses_[3].second;
  ofs << "\n\n";*/
//}

void vizApp::mouseDrag(MouseEvent event){
  // let the camera handle the interaction
  maya_cam_.mouseDrag(event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown());
}

void vizApp::mouseDown(MouseEvent event){
  maya_cam_.mouseDown(event.getPos());
}

void vizApp::keyDown(KeyEvent event){

  //if (event.getChar() == ' '){
  //  load_next_image_ = true;
  //  return;
  //}
  //else if (event.getChar() == 'j'){
  //  draw2 = !draw2;
  //  return;
  //}
  //else if (event.getChar() == 'k'){
  //  draw3 = !draw3;
  //  return;
  //}
  //else if (event.getChar() == 's'){
  //  save_toggle_ = !save_toggle_;
  //  return;
  //}
  //else if (event.getChar() == 'a'){
  //  draw3d = !draw3d;
  //  return;
  //}
  //else if (event.getCode() == KeyEvent::KEY_ESCAPE){
  //  shutdown();
  //}
  //else if (event.getChar() == 'd'){
  //  manual_control_toggle_ = !manual_control_toggle_;
  //}
  //
  //static int current_model_idx = 0;

  //if (std::isdigit(event.getChar())){
  //  std::stringstream ss;
  //  ss << event.getChar();
  //  ss >> current_model_idx;
  //  return;
  //}


  //if (current_model_idx == 0){
  //  applyOffsetToCamera(event);
  //}
  //else{
  //  if (current_model_idx - 1 >= moving_objects_pg_.size()){
  //    console() << "Error, this index is too large for the vector!" << std::endl;
  //    return;
  //  }
  //  boost::shared_ptr< DaVinciPoseGrabber > grabber;
  //  try{
  //    grabber = moving_objects_pg_[current_model_idx-1]->getDVPoseGrabber();
  //    if (grabber.get() == 0x0) return;
  //    applyOffsetToTrackedObject(event, grabber);

  //  }
  //  catch (...){
  //    return;
  //  }


  //}
  // 

}

void vizApp::applyOffsetToCamera(KeyEvent &event){

  /*boost::shared_ptr<std::vector<double> > offsets = camera_pg_->getDVPoseGrabber()->getOffsets();
  boost::shared_ptr<std::vector<double> > base_offsets = camera_pg_->getDVPoseGrabber()->getBaseOffsets();

  double suj_offset = 0.004;
  double j_offset = 0.001;
  if (manual_control_toggle_){
    suj_offset = 0.001;
    j_offset = 0.0004;
  }


  if (offsets->size() == 0)
    return;

  if (event.getChar() == 'q'){
    (*offsets)[0] += suj_offset;
  }
  else if (event.getChar() == 'w'){
    (*offsets)[1] += suj_offset;
  }
  else if (event.getChar() == 'e'){
    (*offsets)[2] += suj_offset;
  }
  else if (event.getChar() == 'r'){
    (*offsets)[3] += suj_offset;
  }
  else if (event.getChar() == 'z'){
    (*offsets)[0] -= suj_offset;
  }
  else if (event.getChar() == 'x'){
    (*offsets)[1] -= suj_offset;
  }
  else if (event.getChar() == 'c'){
    (*offsets)[2] -= suj_offset;
  }
  else if (event.getChar() == 'v'){
    (*offsets)[3] -= suj_offset;
  }
 

  if (event.getChar() == 'Q'){
    (*base_offsets)[0] += j_offset;
  }
  else if (event.getChar() == 'W'){
    (*base_offsets)[1] += j_offset;
  }
  else if (event.getChar() == 'E'){
    (*base_offsets)[2] += j_offset;
  }
  else if (event.getChar() == 'R'){
    (*base_offsets)[3] += j_offset;
  }
  else if (event.getChar() == 'T'){
    (*base_offsets)[4] += j_offset;
  }
  else if (event.getChar() == 'Y'){
    (*base_offsets)[5] += j_offset;
  }

  else if (event.getChar() == 'Z'){
    (*base_offsets)[0] -= j_offset;
  }
  else if (event.getChar() == 'X'){
    (*base_offsets)[1] -= j_offset;
  }
  else if (event.getChar() == 'C'){
    (*base_offsets)[2] -= j_offset;
  }
  else if (event.getChar() == 'V'){
    (*base_offsets)[3] -= j_offset;
  }
  else if (event.getChar() == 'B'){
    (*base_offsets)[4] -= j_offset;
  }
  else if (event.getChar() == 'N'){
    (*base_offsets)[5] -= j_offset;
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
  */
}

//void vizApp::applyOffsetToTrackedObject(KeyEvent &event, boost::shared_ptr<DaVinciPoseGrabber> grabber){
//
//  boost::shared_ptr<std::vector<double> > offsets = grabber->getOffsets();
//  boost::shared_ptr<std::vector<double> > base_offsets = grabber->getBaseOffsets();
//
//  if (offsets->size() == 0)
//    return;
//
//  double suj_offset = 0.004;
//  double j_offset = 0.001;
//  if (manual_control_toggle_){
//    suj_offset = 0.001;
//    j_offset = 0.0004;
//  }
//
//  if (event.getChar() == 'q'){
//    (*offsets)[0] += suj_offset;
//  }
//  else if (event.getChar() == 'w'){
//    (*offsets)[1] += suj_offset;
//  }
//  else if (event.getChar() == 'e'){
//    (*offsets)[2] += suj_offset;
//  }
//  else if (event.getChar() == 'r'){
//    (*offsets)[3] += suj_offset;
//  }
//  else if (event.getChar() == 't'){
//    (*offsets)[4] += suj_offset;
//  }
//  else if (event.getChar() == 'y'){
//    (*offsets)[5] += suj_offset;
//  }
//  else if (event.getChar() == 'u'){
//    (*offsets)[6] += suj_offset;
//  }
//
//  else if (event.getChar() == 'z'){
//    (*offsets)[0] -= suj_offset;
//  }
//  else if (event.getChar() == 'x'){
//    (*offsets)[1] -= suj_offset;
//  }
//  else if (event.getChar() == 'c'){
//    (*offsets)[2] -= suj_offset;
//  }
//  else if (event.getChar() == 'v'){
//    (*offsets)[3] -= suj_offset;
//  }
//  else if (event.getChar() == 'b'){
//    (*offsets)[4] -= suj_offset;
//  }
//  else if (event.getChar() == 'n'){
//    (*offsets)[5] -= suj_offset;
//  }
//  else if (event.getChar() == 'm'){
//    (*offsets)[6] -= suj_offset;
//  }
//
//  if (event.getChar() == 'Q'){
//    (*base_offsets)[0] += j_offset;
//  }
//  else if (event.getChar() == 'W'){
//    (*base_offsets)[1] += j_offset;
//  }
//  else if (event.getChar() == 'E'){
//    (*base_offsets)[2] += j_offset;
//  }
//  else if (event.getChar() == 'R'){
//    (*base_offsets)[3] += j_offset;
//  }
//  else if (event.getChar() == 'T'){
//    (*base_offsets)[4] += j_offset;
//  }
//  else if (event.getChar() == 'Y'){
//    (*base_offsets)[5] += j_offset;
//  }
//
//  else if (event.getChar() == 'Z'){
//    (*base_offsets)[0] -= j_offset;
//  }
//  else if (event.getChar() == 'X'){
//    (*base_offsets)[1] -= j_offset;
//  }
//  else if (event.getChar() == 'C'){
//    (*base_offsets)[2] -= j_offset;
//  }
//  else if (event.getChar() == 'V'){
//    (*base_offsets)[3] -= j_offset;
//  }
//  else if (event.getChar() == 'B'){
//    (*base_offsets)[4] -= j_offset;
//  }
//  else if (event.getChar() == 'N'){
//    (*base_offsets)[5] -= j_offset;
//  }
//
//
//
//  if (offsets != nullptr){
//    ci::app::console() << "Offset = [ ";
//    for (auto i : *offsets){
//      ci::app::console() << i << ", ";
//    }
//    ci::app::console() << "]" << std::endl;
//
//    ci::app::console() << "Base Offset = [ ";
//    for (auto i : *base_offsets){
//      ci::app::console() << i << ", ";
//    }
//    ci::app::console() << "]" << std::endl;
//  }
//}
