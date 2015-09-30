/**

viz - A robotics visualizer specialized for the da Vinci robotic system.
Copyright (C) 2014 Max Allan

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

**/

#include <CinderOpenCV.h>
#include <locale>
#include <cinder/gl/Vbo.h>
#include <opencv2/highgui/highgui.hpp>

#include "../include/config_reader.hpp"
#include "../include/vizApp.hpp"
#include "../include/resources.hpp"

using namespace viz;

const std::string DATASET_TARGET("/Dataset4/");

std::vector<SubWindow *> vizApp::sub_windows_ = std::vector<SubWindow *>();

void vizApp::setupFromConfig(const std::string &path){
  
  running_ = false;

  ConfigReader reader(path);

  std::string root_dir, output_dir, output_dir_this_run;

  try{
    //sanitise
    if (reader.has_element("root-dir")){

      root_dir = reader.get_element("root-dir");
      if (!boost::filesystem::is_directory(root_dir))
        throw std::runtime_error("");

    }
    else{
      //raise error
      throw std::runtime_error("");
    }

    if (reader.has_element("output-dir")){

      output_dir = reader.get_element("output-dir");
      if (!boost::filesystem::is_directory(output_dir))
        throw std::runtime_error("");

    }

    if (reader.has_element("left-input-video") && reader.has_element("right-input-video")) {

      video_left_ = VideoIO(root_dir + "/" + reader.get_element("left-input-video"));
      video_right_ = VideoIO(root_dir + "/" + reader.get_element("right-input-video"));

    }
    else if (reader.has_element("stereo-input-video")){

      stereo_video_ = VideoIO(root_dir + "/" + reader.get_element("stereo-input-video"));

    }
    else{

      throw std::runtime_error("");

    }

    if (reader.has_element("camera-config")){

      camera_.Setup(reader.get_element("root-dir") + "/" + reader.get_element("camera-config"), 1, 1000);

    }
    else{

      throw std::runtime_error("");

    }

    //create new output subdir
    for (int n = 0;; ++n){
      std::stringstream ss;
      ss << reader.get_element("output-dir") << "/output" << n;
      boost::filesystem::path output_dir(ss.str());
      if (!boost::filesystem::is_directory(output_dir)){
        output_dir_this_run = output_dir.string();
        break;
      }
    }


    SubWindow::output_directory = output_dir_this_run;


    if (reader.has_element("moveable-camera")){

      try{

        moveable_camera_.reset(new PoseGrabber(ConfigReader(root_dir + "/" + reader.get_element("moveable-camera")), output_dir_this_run));

      }
      catch (std::runtime_error){

        try{

          moveable_camera_.reset(new DHDaVinciPoseGrabber(ConfigReader(root_dir + "/" + reader.get_element("moveable-camera")), output_dir_this_run));

        }
        catch (std::runtime_error){

        }

      }
    }

    if (reader.has_element("tracked-camera")){

      tracked_camera_.reset(new PoseGrabber(ConfigReader(root_dir + "/" + reader.get_element("tracked-camera")), output_dir_this_run)); //if there is no moveable camera then there won't be a tracked camera

    }

    loadTrackables(reader, output_dir_this_run);

  }
  catch (std::runtime_error){

    camera_image_width_ = 720;
    camera_image_height_ = 576;
    framebuffer_ = gl::Fbo(camera_image_width_, camera_image_height_);

    return;
  }

  camera_image_width_ = camera_.GetLeftCamera().getImageWidth();
  camera_image_height_ = camera_.GetLeftCamera().getImageHeight();
  framebuffer_ = gl::Fbo(camera_image_width_, camera_image_height_);
  
  state.load_one = true;

  running_ = true;

}

void vizApp::runVideoButton(){

  state.load_all = !state.load_all;

}

void vizApp::savePoseButton(){

  state.save_all = !state.save_all;

}

void vizApp::setupGUI(){

  gui_port.Init("GUI", 0, 0, 0.2*getWindowWidth(), 0.5*getWindowHeight(), false);
  editor_port.Init("Editor", 0, 0.5*getWindowHeight(), 0.2*getWindowWidth(), 0.5*getWindowHeight(), false);

  left_eye.Init("Left Eye", gui_port.GetRect().x2, 0, camera_image_width_, camera_image_height_, 720, 576, true);
  right_eye.Init("Right Eye", gui_port.GetRect().x2, left_eye.Height(), camera_image_width_, camera_image_height_, 720, 576, true);

  scene_viewer.Init("3D Viz", left_eye.GetRect().x2, 0, three_dim_viz_width_, three_dim_viz_height_, true);
  trajectory_viewer.Init("Trajectory Viz", left_eye.GetRect().x2, 0.5*getWindowHeight(), three_dim_viz_width_, three_dim_viz_height_, true);

  gui_ = params::InterfaceGl::create(getWindow(), "App parameters", toPixels(Vec2i(100, 100)));

  gui_->addButton("Run Video", std::bind(&vizApp::runVideoButton, this));
  gui_->addButton("Save pose data", std::bind(&vizApp::savePoseButton, this));
  gui_->addButton("Reset 3D Viewer", std::bind(&vizApp::resetViewerButton, this));
  gui_->addButton("Quit", std::bind(&vizApp::shutdown, this));

  gui_->addSeparator();

  if (moveable_camera_){
    gui_->addButton("Edit camera pose", std::bind(&vizApp::editPoseButton, this, 0));
  }

  for (size_t i = 0; i < trackables_.size(); ++i){
    std::stringstream ss;
    ss << "Edit instrument " << i << " pose";
    gui_->addButton(ss.str(), std::bind(&vizApp::editPoseButton, this, i+1));
  }


}

void vizApp::resetViewerButton() {

  reset_viz_port_ = true;

}

void vizApp::editPoseButton(const size_t item_idx){

  if (item_idx == 0){

    if (moveable_camera_){
      moveable_camera_->ParamModifier()->show();
    }

    for (size_t i = 0; i < trackables_.size(); ++i){
      trackables_[i]->ParamModifier()->hide();
    }

  }else{
    
    if (moveable_camera_){
      moveable_camera_->ParamModifier()->hide();
    }

    for (size_t i = 0; i < trackables_.size(); ++i){
      if ((item_idx - 1) == i){
        trackables_[i]->ParamModifier()->show();
      }
      else{
        trackables_[i]->ParamModifier()->hide();
      }
    }

  }


}

void vizApp::setup(){

  running_ = false;

  std::vector<std::string> cmd_line_args = getArgs();

  setFullScreen(true);

  state.load_one = false;
  state.load_all = false;
  
  state.save_one = false;
  state.save_all = false;

  reset_viz_port_ = true;

  shader_ = gl::GlslProg(loadResource(RES_SHADER_VERT), loadResource(RES_SHADER_FRAG));

  if (cmd_line_args.size() == 2){

    setupFromConfig(cmd_line_args[1]);
    
  }
  else{
    camera_image_width_ = 720;
    camera_image_height_ = 576;
    framebuffer_ = gl::Fbo(camera_image_width_, camera_image_height_);
  }


  three_dim_viz_width_ = 576;
  three_dim_viz_height_ = 576;
  framebuffer_3d_ = gl::Fbo(three_dim_viz_width_, three_dim_viz_height_);
   
  setupGUI();
 
  shaft_framebuffer = gl::Fbo(camera_image_width_, camera_image_height_);
  head_framebuffer = gl::Fbo(camera_image_width_, camera_image_height_);
  clasper1_framebuffer = gl::Fbo(camera_image_width_, camera_image_height_);
  clasper2_framebuffer = gl::Fbo(camera_image_width_, camera_image_height_);

  clasper1_base_framebuffer = gl::Fbo(camera_image_width_, camera_image_height_);
  clasper2_base_framebuffer = gl::Fbo(camera_image_width_, camera_image_height_);

}

void vizApp::updateModels(){

  if (moveable_camera_){
    if (!moveable_camera_->LoadPose(state.load_one || state.load_all)){
      running_ = false;
      return;
    }
  }

  if (tracked_camera_){
    if (!tracked_camera_->LoadPose(state.load_one || state.load_all)){
      running_ = false;
      return;
    }
  }

  for (size_t i = 0; i < trackables_.size(); ++i){
    if (!trackables_[i]->LoadPose(state.load_one || state.load_all)){
      running_ = false;
      return;
    }
  }

}

void vizApp::updateVideo(){

  if (state.load_one || state.load_all){

    cv::Mat stereo_image;

    cv::Mat left_frame;
    cv::Mat right_frame;

    if (video_left_.IsOpen() && video_right_.IsOpen()){

      left_frame = video_left_.Read();
      right_frame = video_right_.Read();


    }
    else if (stereo_video_.IsOpen()){

      stereo_video_.Read(left_frame, right_frame);

    }


    if ((video_left_.IsOpen() && (!video_left_.CanRead() || !video_right_.CanRead())) || ((stereo_video_.IsOpen() && !stereo_video_.CanRead()))){

      state.load_all = false;
      state.load_one = false;
      running_ = false;

    }
    else{

      cv::Mat resized_left, resized_right;
      
      if (left_frame.size() == cv::Size(0, 0)){
        left_frame = cv::Mat::zeros(cv::Size(camera_.GetLeftCamera().getImageWidth(), camera_.GetLeftCamera().getImageHeight()), CV_8UC3);
      }

      if (right_frame.size() == cv::Size(0, 0)){
        right_frame = cv::Mat::zeros(cv::Size(camera_.GetLeftCamera().getImageWidth(), camera_.GetLeftCamera().getImageHeight()), CV_8UC3);
      }

      cv::resize(left_frame, resized_left, cv::Size(camera_.GetLeftCamera().getImageWidth(), camera_.GetLeftCamera().getImageHeight()));
      cv::resize(right_frame, resized_right, cv::Size(camera_.GetRightCamera().getImageWidth(), camera_.GetRightCamera().getImageHeight()));
      left_texture_ = fromOcv(resized_left);
      right_texture_ = fromOcv(resized_right);

    }

  }

}

void vizApp::update(){
 
  if (!running_) return;

  updateModels();

  updateVideo();

}

void vizApp::draw2D(gl::Texture &tex){
  
  if (!tex) return;

  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);
  glViewport(0, 0, camera_image_width_, camera_image_height_);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  
  glLoadIdentity();
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

void vizApp::drawLeftEye() {

  if (!running_) return;

  drawEye(left_texture_, true);
  
}

void vizApp::drawRightEye(){

  if (!running_) return;

  drawEye(right_texture_, false);

}

void vizApp::draw(){
  
  gl::clear(Color(0, 0, 0));

  /** draw GUI **/
  gui_port.Draw(gui_);

  if (!running_) return;

  /** draw editor **/
  if (moveable_camera_){
    editor_port.Draw(moveable_camera_->ParamModifier());
  }
  
  for (size_t i = 0; i < trackables_.size(); ++i){
    editor_port.Draw(trackables_[i]->ParamModifier());
  }

  /** draw left eye **/
  left_eye.BindAndClear();
  drawLeftEye();
  left_eye.UnBind();
  left_eye.Draw();

  /** draw right eye **/
  right_eye.BindAndClear();
  drawRightEye();
  right_eye.UnBind();
  right_eye.Draw();

  /** draw scene **/
  scene_viewer.BindAndClear();
  drawScene(left_texture_, right_texture_);
  scene_viewer.UnBind();
  scene_viewer.Draw();

  trajectory_viewer.BindAndClear();
  drawCameraTracker();
  trajectory_viewer.UnBind();
  trajectory_viewer.Draw();

  ////////////

  if (state.load_one || state.load_all){
    drawSegmentation();
    draw2DTrack();
  }
  ///////////

  saveState();

  state.load_one = false;
  state.save_one = false;

}

void vizApp::draw3DViewports(){

  framebuffer_3d_.bindFramebuffer();
  drawScene(left_texture_, right_texture_);
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
  
  drawCamera(gl::Texture(), gl::Texture());

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

void vizApp::drawCamera(gl::Texture &left_image_data, gl::Texture &right_image_data){
  
  ci::Vec3f left_cam_center(0, 0, 0);
  ci::Vec3f left_cam_bottom_left(-5, -5, 20);
  ci::Vec3f left_cam_bottom_right(5, -5, 20);
  ci::Vec3f left_cam_top_left(-5, 5, 20);
  ci::Vec3f left_cam_top_right(5, 5, 20);

  ci::Vec3f left_to_right_translate(11, 0, 0);
  ci::Vec3f right_cam_center = left_cam_center + left_to_right_translate;
  ci::Vec3f right_cam_bottom_left = left_cam_bottom_left + left_to_right_translate;
  ci::Vec3f right_cam_bottom_right = left_cam_bottom_right + left_to_right_translate;
  ci::Vec3f right_cam_top_left = left_cam_top_left + left_to_right_translate;
  ci::Vec3f right_cam_top_right = left_cam_top_right + left_to_right_translate;
 
  if (left_image_data && right_image_data){
    drawImageOnCamera(left_image_data, left_cam_top_left, left_cam_bottom_left, left_cam_top_right, left_cam_bottom_right);
    drawImageOnCamera(right_image_data, right_cam_top_left, right_cam_bottom_left, right_cam_top_right, right_cam_bottom_right);
  }

  ci::Vec3f vertex[8];

  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, 0, &vertex[0].x);

  vertex[0] = left_cam_center;
  vertex[1] = left_cam_bottom_left;
  vertex[2] = left_cam_center;
  vertex[3] = left_cam_bottom_right;
  vertex[4] = left_cam_center;
  vertex[5] = left_cam_top_left;
  vertex[6] = left_cam_center;
  vertex[7] = left_cam_top_right;

  glDrawArrays(GL_LINES, 0, 8);

  vertex[0] = right_cam_center;
  vertex[1] = right_cam_bottom_left;
  vertex[2] = right_cam_center;
  vertex[3] = right_cam_bottom_right;
  vertex[4] = right_cam_center;
  vertex[5] = right_cam_top_left;
  vertex[6] = right_cam_center;
  vertex[7] = right_cam_top_right;

  glDrawArrays(GL_LINES, 0, 8);

  glLineWidth(2.0f);
  vertex[0] = left_cam_bottom_left;
  vertex[1] = left_cam_bottom_right;
  vertex[2] = left_cam_top_right;
  vertex[3] = left_cam_top_left;
  glDrawArrays(GL_LINE_LOOP, 0, 4);

  vertex[0] = right_cam_bottom_left;
  vertex[1] = right_cam_bottom_right;
  vertex[2] = right_cam_top_right;
  vertex[3] = right_cam_top_left;
  glDrawArrays(GL_LINE_LOOP, 0, 4);

  glLineWidth(1.0f);
  glDisableClientState(GL_VERTEX_ARRAY);
  
}

void vizApp::drawCameraTracker(){

  gl::clear(Color(0.0, 0.0, 0.0));

  if (!running_) return;

  if (!moveable_camera_ || !tracked_camera_) return;
  if (moveable_camera_->History().size() == 0 || tracked_camera_->History().size() == 0) return;
  
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

void vizApp::drawScene(gl::Texture &left_image, gl::Texture &right_image){
  
  if (!running_) return;

  gl::clear(Color(0.15, 0.15, 0.15));

  if (trackables_.size() == 0) return;
  
  if (reset_viz_port_){

    ci::Vec3f eye_point = trackables_[0]->GetPose().getTranslate().xyz() + ci::Vec3f(77.7396, -69.9107, -150.47f);

    ci::CameraPersp maya;
    maya.setEyePoint(eye_point);
    maya.setOrientation(ci::Quatf(ci::Vec3f(0.977709, -0.0406959, 0.205982), 2.75995));
    maya_cam_2_.setCurrentCam(maya);
    reset_viz_port_ = false;
  
  }

  gl::pushMatrices();
  gl::setMatrices(maya_cam_2_.getCamera());
  

  ci::Area viewport = gl::getViewport();
  gl::setViewport(ci::Area(0, 0, framebuffer_3d_.getWidth(), framebuffer_3d_.getHeight()));

  if (moveable_camera_){

    gl::pushModelView();
    gl::multModelView(moveable_camera_->GetPose());
    camera_.TurnOnLight();
    drawCamera(left_image, right_image);
    gl::popModelView();

  }
  else{
    camera_.TurnOnLight();
    drawCamera(left_image, right_image);
  }
  

  if (tracked_camera_){

    gl::pushModelView();
    gl::multModelView(tracked_camera_->GetPose());
    drawCamera(left_image, right_image);
    gl::popModelView();

  }

  
  shader_.bind();
  shader_.uniform("tex0", 0);

  for (size_t i = 0; i < trackables_.size(); ++i){

    trackables_[i]->GetPose(); //update the pose if needed
    trackables_[i]->Draw();

  }

  shader_.unbind();

  //if (moveable_camera_)
  camera_.TurnOffLight();

  gl::setViewport(viewport);
  gl::popMatrices();




}

void vizApp::savePoses(){
  
  ci::Matrix44f camera_pose;
  camera_pose.setToIdentity();

  //need to use the camera pose, if we have one (i.e. it's not just identity) to set the instrument pose
  if (moveable_camera_){
    moveable_camera_->WritePoseToStream();
    camera_pose = moveable_camera_->GetPose();
  }

  for (std::size_t i = 0; i < trackables_.size(); ++i){
    trackables_[i]->WritePoseToStream(camera_pose);
  }

  if (tracked_camera_){
    tracked_camera_->WritePoseToStream();
  }

}

void vizApp::saveState(){

  if (!(state.load_one || state.load_all)) return;

  if (state.save_all || state.save_one){
    for (auto sw : sub_windows_){
      if (sw->IsSaving()) sw->WriteFrameToFile();
    }
    savePoses();
    state.save_one = false;
  }

}

void vizApp::shutdown(){

  for (size_t i = 0; i < trackables_.size(); ++i){

    trackables_[i].reset();

  }

  if (moveable_camera_) moveable_camera_.reset();
  if (tracked_camera_) tracked_camera_.reset();

  for (auto sw : sub_windows_){
    if (sw->CanSave()) sw->CloseStream();
  }

  video_left_.CloseStreams();
  video_right_.CloseStreams();

  running_ = false;

  AppNative::shutdown();
  AppNative::quit();

}

void vizApp::drawTargets(){

  for (size_t i = 0; i < trackables_.size(); ++i){

    trackables_[i]->Draw();

  }

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

  draw2D(texture);

  gl::enableDepthRead();
  gl::enableDepthWrite();
  gl::pushMatrices();

  if (is_left){
    camera_.setupLeftCamera(maya_cam_, getCameraPose()); //do viewport and set camera pose
  }
  else{
    camera_.setupRightCamera(maya_cam_, getCameraPose());
  }

  shader_.bind();
  shader_.uniform("tex0", 0);

  drawTargets();

  shader_.unbind();

  gl::popMatrices();

  camera_.unsetCameras(); //reset the viewport values

}

cv::Mat SetMatFromTexture(const cv::Mat &tex, char val){

  return (tex != 1)*val;

}

cv::Mat CombineShaftAndHead(const cv::Mat &body, const cv::Mat &head, unsigned char val_body, unsigned char val_head){

  cv::Mat combined = cv::Mat::zeros(body.size(), CV_8UC1);

  for (int r = 0; r < body.rows; ++r){
    for (int c = 0; c < body.cols; ++c){

      float body_depth = body.at<float>(r, c);
      float head_depth = head.at<float>(r, c);

      if (body_depth == 1 && head_depth == 1) continue;

      if (head_depth != 1){
        combined.at<unsigned char>(r, c) = val_head;
      }

      if (body_depth != 1){
        combined.at<unsigned char>(r, c) = val_body;
      }

      /*if (body_depth == 1){
        combined.at<unsigned char>(r, c) = val_head;
      }
      else if (head_depth == 1){
        combined.at<unsigned char>(r, c) = val_body;
      }
      else{
        if (body_depth < head_depth){
          combined.at<unsigned char>(r, c) = val_body;
        }
        else{
          combined.at<unsigned char>(r, c) = val_head;
        }
      }*/
    }
  }
  /*
  std::vector< std::vector< cv::Point> > shaft_contours;
  std::vector< std::vector< cv::Point> > head_contours;

  cv::Mat shaft = (combined == val_body) * 255;
  cv::Mat clasper = (combined == val_head) * 255;

  cv::Mat shaft_edges = cv::Mat::zeros(shaft.size(), CV_8UC1), clasper_edges = cv::Mat::zeros(clasper.size(), CV_8UC1);
  for (int r = 1; r < shaft.rows - 1; ++r){
    for (int c = 1; c < shaft.cols - 1; ++c){
      if (shaft.at<unsigned char>(r, c) == 255){
        if (shaft.at<unsigned char>(r + 1, c) == 0 || shaft.at<unsigned char>(r - 1, c) == 0 || shaft.at<unsigned char>(r, c + 1) == 0 || shaft.at<unsigned char>(r, c - 1) == 0){
          shaft_edges.at<unsigned char>(r, c) = 255;
        }
      }

      if (clasper.at<unsigned char>(r, c) == 255){
        if (clasper.at<unsigned char>(r + 1, c) == 0 || clasper.at<unsigned char>(r - 1, c) == 0 || clasper.at<unsigned char>(r, c + 1) == 0 || clasper.at<unsigned char>(r, c - 1) == 0){
          clasper_edges.at<unsigned char>(r, c) = 255;
        }
      }

    }
  }

  cv::findContours(shaft_edges, shaft_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  cv::findContours(shaft_edges, head_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  std::sort(head_contours.begin(), head_contours.end(), [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b){
    return a.size() > b.size();
  });

  std::vector<cv::Point> &shaft_contour = shaft_contours[0];
  std::vector<cv::Point> &head_contour = head_contours[0];
  std::vector<cv::Point> &thing = head_contours[1];

  cv::Mat mask = cv::Mat::zeros(combined.rows + 2, combined.cols + 2, CV_8UC1);
  std::vector<std::vector<cv::Point> >t;
  t.push_back(thing);
  cv::drawContours(mask, t, -1, cv::Scalar(255), CV_FILLED, 8);

  std::vector<cv::Point> to_fill_vals;

  for (int r = 1; r < mask.rows-1; ++r){
    for (int c = 1; c < mask.cols-1; ++c){
      if (mask.at<unsigned char>(r, c) == 255){
        to_fill_vals.push_back(cv::Point(c - 1, r - 1));
      }
    }
  }

  for (const auto &pt : to_fill_vals){
    combined.at<unsigned char>(pt) = val_body;
  }*/

  return combined;

}

void vizApp::drawSegmentation(){

	static bool first = true;
	static gl::Fbo framebuffer(camera_image_width_, camera_image_height_);
	static cv::VideoWriter cap("PSM1_Segmentation.avi", CV_FOURCC('D', 'I', 'B', ' '), 25, cv::Size(camera_image_width_, camera_image_height_));
	
	framebuffer.bindFramebuffer();

	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::pushMatrices();

	camera_.setupLeftCamera(maya_cam_, getCameraPose()); //do viewport and set camera pose
	boost::shared_ptr<DHDaVinciPoseGrabber> lnd = boost::dynamic_pointer_cast<DHDaVinciPoseGrabber>(trackables_[0]);

  gl::clear();
	lnd->DrawBody();
	cv::Mat body = toOcv(framebuffer.getDepthTexture());

	gl::clear();
  lnd->DrawHead();
  cv::Mat head = toOcv(framebuffer.getDepthTexture());

  cv::Mat combined = CombineShaftAndHead(body, head, 160, 70);

  gl::popMatrices();
  camera_.unsetCameras(); //reset the viewport values

	framebuffer.unbindFramebuffer();

  cv::Mat combined_flipped;
  cv::flip(combined, combined_flipped, 0);
  cap << combined_flipped;

}

ci::Vec2f GetEnd(cv::Mat &image, bool IS_PSM_1){

  std::vector<cv::Point> line;
  for (int r = 0; r < image.rows; ++r){
    for (int c = 0; c < image.cols; ++c){
      if (image.at<cv::Vec4b>(r, c)[0] != 0.0){
        line.push_back(cv::Point(c, r));
      }
    }
  }

  if (line.size() == 0) return ci::Vec2f(-1, -1);

  //sort so that the right most values are first and the left most are last
  std::sort(line.begin(), line.end(), [](cv::Point a, cv::Point b){ return a.x > b.x; });

  if (IS_PSM_1){
    //return ci::Vec2f(line.back().x, line.back().y);
    return ci::Vec2f(line.front().x, line.front().y);

  }
  else{
    //return ci::Vec2f(line.front().x, line.front().y);
    return ci::Vec2f(line.back().x, line.back().y);
  }

}

ci::Vec2f GetCenter(cv::Mat &image, bool IS_PSM_1){

	std::vector<cv::Point> line;
	for (int r = 0; r < image.rows; ++r){
		for (int c = 0; c < image.cols; ++c){
			if (image.at<cv::Vec4b>(r, c)[0] != 0.0){
				line.push_back(cv::Point(c, r));
			}
		}
	}

  if (line.size() == 0) return ci::Vec2f(-1, -1);

	std::sort(line.begin(), line.end(), [](cv::Point a, cv::Point b){ return a.x > b.x; });

	if (IS_PSM_1){
		return ci::Vec2f(line.front().x, line.front().y);
	}
	else{
		return ci::Vec2f(line.back().x, line.back().y);
	}

}

ci::Vec2f GetVector(cv::Mat &image, bool IS_PSM_1){

	std::vector<cv::Point> line;
	for (int r = 0; r < image.rows; ++r){
		for (int c = 0; c < image.cols; ++c){
			if (image.at<cv::Vec4b>(r, c)[0] != 0.0){
				line.push_back(cv::Point(c, r));
			}
		}
	}

  if (line.size() == 0) return ci::Vec2f(0, 0);

	std::sort(line.begin(), line.end(), [](cv::Point a, cv::Point b){ return a.x > b.x; });

	cv::Point v = line.front() - line.back();
	cv::Vec2f vf((float)v.x / std::sqrt(v.x*v.x + v.y*v.y), (float)v.y / std::sqrt(v.x*v.x + v.y*v.y));
	
	if (IS_PSM_1)
		return ci::Vec2f(vf[0], vf[1]);
	else
		return -ci::Vec2f(vf[0], vf[1]);

}

ci::Vec2i GetCOM(cv::Mat &image){

	cv::Vec2f com;
	size_t count = 0;
	for (int r = 0; r < image.rows; ++r){
		for (int c = 0; c < image.cols; ++c){
			if (image.at<cv::Vec4b>(r, c)[0] != 0.0){
				com += cv::Vec2f(c, r);
				count++;
			}
		}
	}

  if (count == 0) return ci::Vec2f(-1, -1);

	return ci::Vec2i(com[0] / count, com[1] / count);


}

ci::Vec2i vizApp::GetEndOfShaft(ci::Vec2f &shaft_start, ci::Vec2f &shaft_end){

  cv::Mat frame = left_eye.getFrame();
  cv::Mat ff; cv::flip(frame, ff, 0);

  std::vector<std::pair<cv::Vec4b,cv::Point>> colors;

  for (float d = 0.02; d < 1.0; d += 0.02){
    ci::Vec2f point = shaft_start + d*(shaft_end - shaft_start);

    cv::Point pt(point[0], point[1]);
    colors.push_back(std::make_pair(ff.at<cv::Vec4b>(pt), pt));

  }

  float max_diff = std::numeric_limits<float>::min();
  cv::Point transition_point(-1, -1);
  for (size_t i = 1; i < colors.size(); ++i){

    if (colors[i].first == cv::Vec4b(0, 0, 0, 255) && colors[i - 1].first != cv::Vec4b(0, 0, 0, 255) ){
      transition_point = colors[i].second;
    }

    else if (colors[i-1].first == cv::Vec4b(0, 0, 0, 255) && colors[i].first != cv::Vec4b(0, 0, 0, 255)){
      transition_point = colors[i].second;
    }

    //float diff = (std::abs(float(colors[i].first[0]) - float(colors[i - 1].first[0])) + std::abs(float(colors[i].first[2]) - float(colors[i - 1].first[2])) + std::abs(float(colors[i].first[2]) - float(colors[i - 1].first[2])))/3;

    //if (diff > max_diff){
    //  max_diff = diff;
    //  transition_point = colors[i].second;
    //}

  }

  return ci::Vec2i(transition_point.x, transition_point.y);

}

void vizApp::draw2DTrack(){

	//static bool first = true;
	
	static std::ofstream ofs("PSM1_Pose.txt");
    
  cv::Mat frame;
  cv::flip(left_eye.getFrame(), frame, 0);
  cv::Mat camera_image = toOcv(left_texture_);

	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::pushMatrices();
  glDisable(GL_TEXTURE_2D);

	camera_.setupLeftCamera(maya_cam_, getCameraPose()); //do viewport and set camera pose
  boost::shared_ptr<SE3DaVinciPoseGrabber> lnd = boost::dynamic_pointer_cast<SE3DaVinciPoseGrabber>(trackables_[0]);
	ci::Matrix44f shaft_pose, head_pose, clasper_left_pose, clasper_right_pose;
	shaft_pose = lnd->GetPose();
	lnd->GetModelPose(head_pose, clasper_left_pose, clasper_right_pose);

  //shaft 
	shaft_framebuffer.bindFramebuffer();
	gl::clear();
  gl::color(1.0, 1.0, 1.0);
  ci::Vec3f origin(0, 0, 0);
	ci::Vec3f shaft_dir(0, 0, -25);
  gl::pushModelView();
  ci::gl::multModelView(shaft_pose);
  gl::drawLine(origin, shaft_dir);
  gl::popModelView();
  shaft_framebuffer.unbindFramebuffer();
	glFinish();

  //head
  head_framebuffer.bindFramebuffer();
	gl::clear();
  gl::color(1.0, 1.0, 1.0);
	gl::pushModelView();
	gl::multModelView(head_pose);
	gl::drawSphere(ci::Vec3f(0, 0, 0), 3);
  head_framebuffer.unbindFramebuffer();
	gl::popModelView();
	glFinish();

  //clasper 1 center
  clasper1_framebuffer.bindFramebuffer();
  gl::color(1.0, 1.0, 1.0);
  gl::clear();
	gl::pushModelView();
  gl::multModelView(clasper_left_pose);
  gl::drawLine(ci::Vec3f(0, 0, 0), ci::Vec3f(2.5, 0, 10));
	gl::popModelView();
  clasper1_framebuffer.unbindFramebuffer();
	glFinish();
  
  //clasper 1 base
  clasper1_base_framebuffer.bindFramebuffer();
  gl::color(1.0, 1.0, 1.0);
  gl::clear();
  gl::pushModelView();
  gl::multModelView(clasper_left_pose);
  gl::drawSphere(ci::Vec3f(0, 0, 0), 3);
  gl::popModelView();
  clasper1_base_framebuffer.unbindFramebuffer();
  glFinish();
  
  //claserp 2 center
  clasper2_framebuffer.bindFramebuffer();
  gl::color(1.0, 1.0, 1.0);
  gl::clear();
	gl::pushModelView();
  //lnd->DrawClaspers2();
  gl::multModelView(clasper_right_pose);
  gl::drawLine(ci::Vec3f(0, 0, 0), ci::Vec3f(-2.5, 0, 10));
	gl::popModelView();
  clasper2_framebuffer.unbindFramebuffer();
	glFinish();

  //clasper 2 base
  clasper2_base_framebuffer.bindFramebuffer();
  gl::color(1.0, 1.0, 1.0);
  gl::clear();
  gl::pushModelView();
  gl::multModelView(clasper_right_pose);
  gl::drawSphere(ci::Vec3f(0, 0, 0), 3);
  gl::popModelView();
  clasper2_base_framebuffer.unbindFramebuffer();
  glFinish();
  
	gl::popMatrices();
	camera_.unsetCameras(); //reset the viewport values
  	

  cv::Mat shaft_axis_ = toOcv(shaft_framebuffer.getTexture());
  cv::Mat shaft_axis;
  cv::flip(shaft_axis_, shaft_axis, 0);
  cv::Mat head_ = toOcv(head_framebuffer.getTexture());
  cv::Mat head;
  cv::flip(head_, head, 0);
  cv::Mat clasper_left_ = toOcv(clasper1_framebuffer.getTexture());
  cv::Mat clasper_left;
  cv::flip(clasper_left_, clasper_left, 0);
  cv::Mat clasper_right_ = toOcv(clasper2_framebuffer.getTexture());
  cv::Mat clasper_right;
  cv::flip(clasper_right_, clasper_right, 0);
  cv::Mat clasper_left_base_ = toOcv(clasper1_base_framebuffer.getTexture());
  cv::Mat clasper_right_base_ = toOcv(clasper2_base_framebuffer.getTexture());
  cv::Mat clasper_left_base, clasper_right_base;
  cv::flip(clasper_left_base_, clasper_left_base, 0); cv::flip(clasper_right_base_, clasper_right_base, 0);

  ci::Vec2f center_of_head = GetCOM(head);
  ci::Vec2f start_of_shaft = GetEnd(shaft_axis, true);
  ci::Vec2f instrument_tracked_point = GetEndOfShaft(center_of_head, start_of_shaft);
  
	ci::Vec2f center_of_l_clasper = GetCOM(clasper_left);
	ci::Vec2f center_of_r_clasper = GetCOM(clasper_right);
  ci::Vec2f base_of_l_clasper = GetCOM(clasper_left_base);
  ci::Vec2f base_of_r_clasper = GetCOM(clasper_right_base);
  
  cv::Mat all_frame = camera_image.clone();


  //draw line along the axis of the the instrument
  ci::Vec2f unit_vector_along_shaft(-1, -1);
  if (start_of_shaft != ci::Vec2f(-1, -1) && center_of_head != ci::Vec2f(-1, -1)){
    cv::line(all_frame, cv::Point(start_of_shaft[0], start_of_shaft[1]), cv::Point(instrument_tracked_point[0], instrument_tracked_point[1]), cv::Scalar(255, 0, 0), 2);
    unit_vector_along_shaft = instrument_tracked_point - start_of_shaft;
    unit_vector_along_shaft.normalize();
  }
  
  //mark the tracked point
  if (instrument_tracked_point != ci::Vec2f(-1, -1)){
    cv::circle(all_frame, cv::Point(instrument_tracked_point[0], instrument_tracked_point[1]), 8, cv::Scalar(255, 0, 0), 1);
  }

  //mark the center of the head
  if (center_of_head != ci::Vec2f(-1, -1) && instrument_tracked_point != ci::Vec2f(-1,-1)){
    cv::circle(all_frame, cv::Point(center_of_head[0], center_of_head[1]), 8, cv::Scalar(255, 0, 0), 1);
    cv::line(all_frame, cv::Point(instrument_tracked_point[0], instrument_tracked_point[1]), cv::Point(center_of_head[0], center_of_head[1]), cv::Scalar(255, 0, 0) );
  }

  if (center_of_head != ci::Vec2f(-1, -1) && base_of_l_clasper != ci::Vec2f(-1,-1)){
    cv::line(all_frame, cv::Point(center_of_head[0], center_of_head[1]), cv::Point(base_of_l_clasper[0], base_of_l_clasper[1]), cv::Scalar(255, 255, 0), 1);
  }

  ci::Vec2f unit_vector_to_clasper(-1, -1);
  //draw the lines to the end of the claspers and stick circles there.
  if (base_of_l_clasper != ci::Vec2f(-1, -1) && center_of_l_clasper != ci::Vec2f(-1, -1)){
    cv::line(all_frame, cv::Point(center_of_l_clasper[0], center_of_l_clasper[1]), cv::Point(base_of_l_clasper[0], base_of_l_clasper[1]), cv::Scalar(255, 0, 0), 2);
    cv::circle(all_frame, cv::Point(center_of_l_clasper[0], center_of_l_clasper[1]), 8, cv::Scalar(0, 0, 255), 1);
    unit_vector_to_clasper = base_of_l_clasper - center_of_head;
    unit_vector_to_clasper.normalize();
  }

  if (base_of_r_clasper != ci::Vec2f(-1, -1) && center_of_r_clasper != ci::Vec2f(-1, -1)){
    cv::line(all_frame, cv::Point(center_of_r_clasper[0], center_of_r_clasper[1]), cv::Point(base_of_r_clasper[0], base_of_r_clasper[1]), cv::Scalar(255, 0, 0), 2);
    cv::circle(all_frame, cv::Point(center_of_r_clasper[0], center_of_r_clasper[1]), 8, cv::Scalar(0, 255, 0), 1);  

  }

  float angle_between_clasper = -1;
  if (base_of_r_clasper != ci::Vec2f(-1, -1) && center_of_r_clasper != ci::Vec2f(-1, -1) && base_of_l_clasper != ci::Vec2f(-1, -1) && center_of_l_clasper != ci::Vec2f(-1, -1)){
    ci::Vec2f a_normed = center_of_r_clasper - base_of_r_clasper;
    ci::Vec2f b_normed = center_of_l_clasper - base_of_l_clasper;
    a_normed.normalize(); b_normed.normalize();

    float dot = a_normed[0] * b_normed[0] + a_normed[1] * b_normed[1];
    if (dot > 1 || dot < -1){
      angle_between_clasper = 0;
    }
    else{
      angle_between_clasper = acos(dot) * 180 / 3.141592;
      angle_between_clasper -= 5;
      if (angle_between_clasper < 0) angle_between_clasper = 0;
    }
  }

  glEnable(GL_TEXTURE_2D);

  std::stringstream to_write;

  //write the tracked point
  //the unit vector pointing along the axis

  to_write << instrument_tracked_point[0] << " " << instrument_tracked_point[1] << " " << unit_vector_along_shaft[0] << " " << unit_vector_along_shaft[1] << " ";
  to_write << unit_vector_to_clasper[0] << " " << unit_vector_to_clasper[1] << " " << angle_between_clasper;

  cv::Mat TEST_OUTPUT = camera_image.clone();
  cv::circle(TEST_OUTPUT, cv::Point(instrument_tracked_point[0], instrument_tracked_point[1]), 10, cv::Scalar(0, 0, 255));
  cv::line(TEST_OUTPUT, cv::Point(instrument_tracked_point[0], instrument_tracked_point[1]), cv::Point2f(instrument_tracked_point[0], instrument_tracked_point[1]) + 40 * cv::Point2f(unit_vector_along_shaft[0], unit_vector_along_shaft[1]), cv::Scalar(255, 0, 0), 1);
  
  cv::Point a = cv::Point(center_of_head[0], center_of_head[1]);
  cv::Point2f b = cv::Point2f(center_of_head[0], center_of_head[1]) + (40 * cv::Point2f(unit_vector_to_clasper[0], unit_vector_to_clasper[1]));
  cv::line(TEST_OUTPUT, a, b, cv::Scalar(0, 255, 0));

  std::stringstream ss;
  ss << angle_between_clasper;
  cv::putText(TEST_OUTPUT, ss.str(), cv::Point(center_of_head[0], center_of_head[1]) + 20 * cv::Point(unit_vector_to_clasper[0], unit_vector_to_clasper[1]), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
  
  static cv::VideoWriter vwriter("PSM1.avi", CV_FOURCC('D', 'I', 'B', ' '), 25, all_frame.size());
  vwriter << TEST_OUTPUT;

  ofs << to_write.str() << "\n";
  ofs.flush();

}

void vizApp::loadTrackables(const ConfigReader &reader, const std::string &output_dir_this_run){

  for (int i = 0;; ++i){

    try{

      std::stringstream s; 
      s << "trackable-" << i;
      loadTrackable(reader.get_element("root-dir") + "/" + reader.get_element(s.str()), output_dir_this_run);
      break;

    }
    catch (std::runtime_error){
      break;
    }

  }

}

void vizApp::loadTrackable(const std::string &filepath, const std::string &output_dir){

  ConfigReader reader(filepath);
  try{
    trackables_.push_back(boost::shared_ptr<BasePoseGrabber>(new DHDaVinciPoseGrabber(reader, output_dir)));
    return;
  }
  catch (std::runtime_error){

  }
  try{
    trackables_.push_back(boost::shared_ptr<BasePoseGrabber>(new SE3DaVinciPoseGrabber(reader, output_dir)));
    return;
  }
  catch (std::runtime_error){

  }
  try{
    trackables_.push_back(boost::shared_ptr<BasePoseGrabber>(new PoseGrabber(reader, output_dir)));
    return;
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

void vizApp::mouseDrag(MouseEvent event){
  // let the camera handle the interaction
  mouse_pos_ = event.getPos();
  maya_cam_2_.mouseDrag(event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown());
}

void vizApp::mouseDown(MouseEvent event){
  maya_cam_2_.mouseDown(event.getPos());
}

void vizApp::fileDrop(FileDropEvent event){

  try{

    for (int i = 0; i < event.getNumFiles(); ++i)
      setupFromConfig(event.getFile(i).string());
  }
  catch (std::runtime_error){
    ci::app::console() << "Error loading from this config file\n";
  }

}

void vizApp::mouseMove(MouseEvent m_event){
  // keep track of the mouse
  mouse_pos_ = m_event.getPos();
}

void vizApp::keyDown(KeyEvent event){

  if (event.getChar() == ' '){
    state.load_one = true;
    return;
  }

  //else if (event.getChar() == 's'){
  //  save_toggle_ = !save_toggle_;
  //  return;
  //}
  //
  //else if (event.getCode() == KeyEvent::KEY_ESCAPE){
  //  shutdown();
  //}
  //
  //else if (event.getChar() == 'd'){
  //  save_toggle_ = !save_toggle_;
  //  run_video_ = !run_video_;
  //}

  //else if (event.getChar() == 'f'){
  //  update_toggle_ = !update_toggle_;
  //}

  //else if (event.getChar() == 'l'){
  //  synthetic_save_ = !synthetic_save_;
  //}

  //else if (event.getChar() == 'p'){
  //  save_viewport_data_ = true;
  //}


}
