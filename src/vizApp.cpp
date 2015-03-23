#include <CinderOpenCV.h>
#include <locale>
#include <cinder/gl/Vbo.h>

#include <opencv2/highgui/highgui.hpp>

#include "../include/config_reader.hpp"
#include "../include/vizApp.hpp"
#include "../include/resources.hpp"

using namespace viz;

/**
* Usage guide: 
*
* Space to move to the next frame of video, loading the next pose value from the pose files.
* 's' to save the current frame of video and the computed pose values.
* 'd' to save the current frame of video and then load the next frame until 'd' is hit again.
* 'l' to work with a synthetic video where the movement at each frame is done entirely from 
* keyboard updates
*
*/

void vizApp::setupFromConfig(const std::string &path){
  
  ConfigReader reader(path);

  if (!boost::filesystem::is_directory(reader.get_element("root-dir")))
    throw std::runtime_error("Error, cannot file config dir!");

  if (!boost::filesystem::is_directory(reader.get_element("output-dir"))){
    boost::filesystem::create_directory(reader.get_element("output-dir"));
  }

  std::string output_dir_this_run;
  for (int n = 0; n < 100; ++n){
    std::stringstream ss;
    ss << reader.get_element("output-dir") << "/output" << n;
    boost::filesystem::path output_dir(ss.str());
    if (!boost::filesystem::is_directory(output_dir)){
      boost::filesystem::create_directory(output_dir);
      output_dir_this_run = output_dir.string();
      break;
    }
  }

  if (output_dir_this_run == "")
    throw std::runtime_error("Error, this should not be empty!");

  try{
    video_left_ = VideoIO(reader.get_element("root-dir") + "/" + reader.get_element("left-input-video"), output_dir_this_run + "/" + reader.get_element("left-output-video"));
    video_right_ = VideoIO(reader.get_element("root-dir") + "/" + reader.get_element("right-input-video"), output_dir_this_run + "/" + reader.get_element("right-output-video"));
  } catch (...){
    stereo_video_ = VideoIO(reader.get_element("root-dir") + "/" + reader.get_element("stereo-input-video"), output_dir_this_run + "/" + reader.get_element("stereo-output-video"));
  }



  camera_.Setup(reader.get_element("root-dir") + "/" + reader.get_element("camera-config"), reader.get_element_as_int("window-width"), reader.get_element_as_int("window-height"), 1, 1000);

  try{
    moveable_camera_.reset(new PoseGrabber(ConfigReader(reader.get_element("root-dir") + "/" + reader.get_element("moveable-camera")), output_dir_this_run));
    tracked_camera_.reset(new PoseGrabber(ConfigReader(reader.get_element("root-dir") + "/" + reader.get_element("tracked-camera")), output_dir_this_run)); //if there is no moveable camera then there won't be a tracked camera
  }
  catch (std::runtime_error){
    try{
      moveable_camera_.reset(new DHDaVinciPoseGrabber(ConfigReader(reader.get_element("root-dir") + "/" + reader.get_element("moveable-camera")), output_dir_this_run));
      tracked_camera_.reset(new PoseGrabber(ConfigReader(reader.get_element("root-dir") + "/" + reader.get_element("tracked-camera")), output_dir_this_run)); //tracked camera will always be a SE3 tracked, not DH.
    }
    catch (std::runtime_error){

    }
  }
  
  try{
    loadTrackables(reader, output_dir_this_run);
  }
  catch (std::runtime_error){

  }

  camera_image_width_ = reader.get_element_as_int("window-width");
  camera_image_height_ = reader.get_element_as_int("window-height");

  three_dim_viz_width_ = reader.get_element_as_int("viz-width");
  three_dim_viz_height_ = reader.get_element_as_int("viz-height");

  writer_.open("z:/vizport.avi", CV_FOURCC('D', 'I', 'B', ' '), 25, cv::Size(three_dim_viz_width_, three_dim_viz_height_));

  framebuffer_ = gl::Fbo(camera_image_width_, camera_image_height_);
  framebuffer_3d_ = gl::Fbo(three_dim_viz_width_, three_dim_viz_height_);

  setWindowSize((2 * framebuffer_.getWidth()), framebuffer_.getHeight() + framebuffer_3d_.getHeight());

  load_next_image_ = true;
  loaded_ = true;
}

void vizApp::setup(){

  std::vector<std::string> cmd_line_args = getArgs();

  run_video_ = false;
  save_toggle_ = false;
  update_toggle_ = false;
  synthetic_save_ = false;
  done_ = false;
  loaded_ = false;
  save_viewport_data_ = false;

  shader_ = gl::GlslProg(loadResource(RES_SHADER_VERT), loadResource(RES_SHADER_FRAG));

  if (cmd_line_args.size() == 2){
    
    try{
      setupFromConfig(cmd_line_args[1]);
      return;
    }
    catch (std::runtime_error){
      ci::app::console() << "Error, input file is bad!\n";
    }

  }
 
  camera_image_width_ = 640;
  camera_image_height_ = 480;
  three_dim_viz_width_ = 500;
  three_dim_viz_height_ = 500;
  
  framebuffer_ = gl::Fbo(camera_image_width_, camera_image_height_);
  framebuffer_3d_ = gl::Fbo(three_dim_viz_width_, three_dim_viz_height_);

  setWindowSize((2 * framebuffer_.getWidth()), framebuffer_.getHeight() + framebuffer_3d_.getHeight());
  load_next_image_ = false;

}

void vizApp::update(){
 
  /**
  * update the trackable object and camera poses.
  * update_toggle_ is used when manipulating the object poses in the UI (to remove constant offsets).
  * this sets internal offset parameters inside the objects so their pose needs to be 'refreshed' to get the offset values.
  */
  if (load_next_image_ || run_video_ || update_toggle_){
    if (moveable_camera_){
      if (!moveable_camera_->LoadPose(update_toggle_)){
        done_ = true;
        shutdown();
        quit();
        return;
      }
    }
    if (tracked_camera_){
      if (!tracked_camera_->LoadPose(update_toggle_)){
        done_ = true;
        shutdown();
        quit();
        return;
      }
    }
    for (size_t i = 0; i < trackables_.size(); ++i){
      if (!trackables_[i]->LoadPose(update_toggle_)){
        done_ = true;
        shutdown();
        quit();
        return;
      }
    }
  }

  if (load_next_image_ || run_video_){

    //static size_t i = 1;
    //ci::app::console() << "Loading frame " << i << std::endl;
    //++i;

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


    if ((video_left_.IsOpen() && (!video_left_.CanRead() || !video_right_.CanRead())) || ((stereo_video_.IsOpen() && !stereo_video_.CanRead())) ){

      run_video_ = false;
      done_ = true;
      shutdown();
      quit();

    }
    else{

      cv::Mat resized_left, resized_right;
      cv::resize(left_frame, resized_left, cv::Size(camera_.GetLeftCamera().getImageWidth(), camera_.GetLeftCamera().getImageHeight()));
      cv::resize(right_frame, resized_right, cv::Size(camera_.GetRightCamera().getImageWidth(), camera_.GetRightCamera().getImageHeight()));

      left_texture_ = fromOcv(resized_left);
      right_texture_ = fromOcv(resized_right);
      load_next_image_ = false;

    }
    
  }



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

  if (done_) return;

  if (!loaded_) return;

  gl::clear( Color( 0, 0, 0 ) ); 

  static size_t count = 0;

  framebuffer_.bindFramebuffer();
  drawEye(left_texture_, true);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf(0.0, (float)framebuffer_.getHeight(), (float)framebuffer_.getWidth(), 0.0));
  
  saveFrame(framebuffer_.getTexture(), true); //remember only saves when save_next_frame_ is true, right call turns this off

  cv::Mat l = toOcv(left_texture_);
  cv::Mat r = toOcv(right_texture_);
  gl::Texture left_copy = fromOcv(l);
  gl::Texture right_copy = fromOcv(r);

  framebuffer_.bindFramebuffer();
  drawEye(right_texture_, false);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf((float)framebuffer_.getWidth(), (float)framebuffer_.getHeight(), 2.0 * framebuffer_.getWidth(), 0.0));
  saveFrame(framebuffer_.getTexture(), false); //remember only saves when save_next_frame_ is true, right call turns this off
  
  framebuffer_3d_.bindFramebuffer();
  draw3D(left_copy, right_copy);
  framebuffer_3d_.unbindFramebuffer();
  gl::draw(framebuffer_3d_.getTexture(), ci::Rectf(0.0, (float)framebuffer_.getHeight() + (float)framebuffer_3d_.getHeight(), (float)framebuffer_3d_.getWidth(), (float)framebuffer_.getHeight())); 

  if (!save_toggle_ && !synthetic_save_){

  }
  else{
    cv::Mat frame = toOcv(framebuffer_3d_.getTexture());
    cv::Mat fframe; cv::flip(frame, fframe, 0);
    //writer_ << fframe;
  }

  
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
  //ci::Vec4<unsigned char> color[8];

  glEnableClientState(GL_VERTEX_ARRAY);
  //glEnableClientState(GL_COLOR_ARRAY);
  glVertexPointer(3, GL_FLOAT, 0, &vertex[0].x);
  //glColorPointer(3, GL_UNSIGNED_BYTE, 0, &color[0].x);

  vertex[0] = left_cam_center;
  vertex[1] = left_cam_bottom_left;
  vertex[2] = left_cam_center;
  vertex[3] = left_cam_bottom_right;
  vertex[4] = left_cam_center;
  vertex[5] = left_cam_top_left;
  vertex[6] = left_cam_center;
  vertex[7] = left_cam_top_right;

  //for (int i = 0; i < 8; ++i) color[i] = ci::Vec3<unsigned char>(255, 255, 255);

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
  //glDisableClientState(GL_COLOR_ARRAY);
  
}

void vizApp::drawCameraTracker(){

  gl::clear(Color(0.0, 0.0, 0.0));

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

void vizApp::draw3D(gl::Texture &left_image, gl::Texture &right_image){
  
  gl::clear(Color(0.15, 0.15, 0.15));
  //gl::clear(Color(1.0, 1.0, 1.0));

  if (trackables_.size() == 0) return;
    
  ci::Vec3f eye_point = trackables_[0]->GetPose().getTranslate().xyz() + ci::Vec3f(77.7396, -69.9107, -150.47f);

  static bool first = true;
  if (first){
    ci::CameraPersp maya;
    maya.setEyePoint(eye_point);
    maya.setOrientation(ci::Quatf(ci::Vec3f(0.977709, -0.0406959, 0.205982), 2.75995));
    maya_cam_2_.setCurrentCam(maya);
  }
  first = false;

  gl::pushMatrices();
  gl::setMatrices(maya_cam_2_.getCamera());
  
  //drawGrid(5000, 1, first_camera_pose.getTranslate().xyz()[2] - 200);

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

void vizApp::saveFrame(gl::Texture texture, bool isLeft){
  
  if (!save_toggle_ && !synthetic_save_){
    return;
  }

  cv::Mat frame = toOcv(texture), flipped;
  cv::flip(frame, flipped, 0);

  //save the video frames
  if (isLeft){

    if (video_left_.IsOpen())
      video_left_.Write(flipped);
    else if (stereo_video_.IsOpen()){
      stereo_video_.Write(flipped, cv::Mat::zeros(flipped.size(), CV_8UC3));
    }

  }
  else{

    //if (video_right_.IsOpen())
    //  video_right_.Write(flipped);

  }


  //save the pose data - only on the left frame as we don't want it done 2x for both
  if (isLeft){

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

  //if we are running the video on save, just keep saving and don't turn it off after each frame
  if (!run_video_){
    save_toggle_ = false;
  }

}

void vizApp::shutdown(){

  for (size_t i = 0; i < trackables_.size(); ++i){

    trackables_[i].reset();

  }

  if (moveable_camera_) moveable_camera_.reset();
  if (tracked_camera_) tracked_camera_.reset();

  video_left_.CloseStreams();
  video_right_.CloseStreams();

  AppNative::shutdown();

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

void vizApp::loadTrackables(const ConfigReader &reader, const std::string &output_dir_this_run){

  for (int i = 0;; ++i){

    try{

      std::stringstream s; 
      s << "trackable-" << i;
      loadTrackable(reader.get_element("root-dir") + "/" + reader.get_element(s.str()), output_dir_this_run);

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

void vizApp::movePose(KeyEvent event){
  
  static int current_model_idx = 0;

  //set the index of the model we want to track
  if (std::isdigit(event.getChar(), std::locale())){
    std::stringstream ss;
    ss << event.getChar();
    ss >> current_model_idx;
    return;
   }


  if (current_model_idx == 0){
    applyOffsetToCamera(event);
  }
  else{
    applyOffsetToTrackedObject(event, current_model_idx);
  }
     
}

void vizApp::mouseMove(MouseEvent m_event){
  // keep track of the mouse
  mouse_pos_ = m_event.getPos();
}

void vizApp::keyDown(KeyEvent event){

  if (event.getChar() == ' '){
    load_next_image_ = true;
    return;
  }

  else if (event.getChar() == 's'){
    save_toggle_ = !save_toggle_;
    return;
  }
  
  else if (event.getCode() == KeyEvent::KEY_ESCAPE){
    shutdown();
  }
  
  else if (event.getChar() == 'd'){
    save_toggle_ = !save_toggle_;
    run_video_ = !run_video_;
  }

  else if (event.getChar() == 'f'){
    update_toggle_ = !update_toggle_;
  }

  else if (event.getChar() == 'l'){
    synthetic_save_ = !synthetic_save_;
  }

  else if (event.getChar() == 'p'){
    save_viewport_data_ = true;
  }

  movePose(event);

}

void vizApp::applyOffsetToCamera(KeyEvent &event){

  double suj_offset = 0.004;
  double j_offset = 0.001;
  
  if (moveable_camera_ == nullptr) return;

  boost::shared_ptr<DHDaVinciPoseGrabber> grabber;
  try{

    grabber = boost::dynamic_pointer_cast<DHDaVinciPoseGrabber>(moveable_camera_);

  }
  catch (boost::bad_lexical_cast &){

    return;

  }

  std::vector<double> &arm_offset = grabber->getArmOffsets();
  std::vector<double> &base_offset = grabber->getBaseOffsets();

  //the base offsets
  if (event.getChar() == 'q'){
    base_offset[0] += suj_offset;
  }
  else if (event.getChar() == 'w'){
    base_offset[1] += suj_offset;
  }
  else if (event.getChar() == 'e'){
    base_offset[2] += suj_offset;
  }
  else if (event.getChar() == 'r'){
    base_offset[3] += suj_offset;
  }
  else if (event.getChar() == 't'){
    base_offset[4] += suj_offset;
  }
  else if (event.getChar() == 'y'){
    base_offset[5] += suj_offset;
  }
  else if (event.getChar() == 'Q'){
    base_offset[0] -= suj_offset;
  }
  else if (event.getChar() == 'W'){
    base_offset[1] -= suj_offset;
  }
  else if (event.getChar() == 'E'){
    base_offset[2] -= suj_offset;
  }
  else if (event.getChar() == 'R'){
    base_offset[3] -= suj_offset;
  }
  else if (event.getChar() == 'T'){
    base_offset[4] -= suj_offset;
  }
  else if (event.getChar() == 'Y'){
    base_offset[5] -= suj_offset;
  }


  //the arm offset
  else if (event.getChar() == 'z'){
    arm_offset[0] += suj_offset;
  }
  else if (event.getChar() == 'x'){
    arm_offset[1] += suj_offset;
  }
  else if (event.getChar() == 'c'){
    arm_offset[2] += suj_offset;
  }
  else if (event.getChar() == 'v'){
    arm_offset[3] += suj_offset;
  }
  else if (event.getChar() == 'Z'){
    arm_offset[0] -= suj_offset;
  }
  else if (event.getChar() == 'X'){
    arm_offset[1] -= suj_offset;
  }
  else if (event.getChar() == 'C'){
    arm_offset[2] -= suj_offset;
  }
  else if (event.getChar() == 'V'){
    arm_offset[3] -= suj_offset;
  }

  console() << "Camera base offset is:\n[";
  for (size_t i = 0; i < base_offset.size(); ++i){
    console() << base_offset[i];
    if (i != base_offset.size() - 1)  console() << ", ";
  }
  console() << "]\n";
  console() << "Camera arm offset is:\n[";
  for (size_t i = 0; i < arm_offset.size(); ++i){
    console() << arm_offset[i];
    if (i != arm_offset.size() - 1)  console() << ", ";
  }
  console() << "]" << std::endl;

}

void vizApp::applyOffsetToTrackedObject(KeyEvent &event, const int current_model_idx){

  //top row is SUJ
  //bottom row is J

  int model_idx = current_model_idx - 1; //as the camera is idx zero

  if ((size_t)model_idx >= trackables_.size()) return;

  boost::shared_ptr<DHDaVinciPoseGrabber> grabber;
  try{
  
     grabber = boost::dynamic_pointer_cast<DHDaVinciPoseGrabber>(trackables_[model_idx]);
  
  }
  catch (boost::bad_lexical_cast &){
    
    return;
  
  }

  if (grabber == nullptr) return;

  std::vector<double> &arm_offset = grabber->getArmOffsets();
  std::vector<double> &base_offset = grabber->getBaseOffsets();

  double suj_offset = 0.0004;
  double j_offset = 0.0001;

  const int SCALE_FACTOR = 5;

  //the base offsets
  if (event.getChar() == 'q'){
    base_offset[0] += suj_offset;
  }
  else if (event.getChar() == 'w'){
    base_offset[1] += suj_offset;
  }
  else if (event.getChar() == 'e'){
    base_offset[2] += suj_offset;
  }
  else if (event.getChar() == 'r'){
    base_offset[3] += suj_offset;
  }
  else if (event.getChar() == 't'){
    base_offset[4] += suj_offset;
  }
  else if (event.getChar() == 'y'){
    base_offset[5] += suj_offset;
  }
  else if (event.getChar() == 'Q'){
    base_offset[0] -= suj_offset;
  }
  else if (event.getChar() == 'W'){
    base_offset[1] -= suj_offset;
  }
  else if (event.getChar() == 'E'){
    base_offset[2] -= suj_offset;
  }
  else if (event.getChar() == 'R'){
    base_offset[3] -= suj_offset;
  }
  else if (event.getChar() == 'T'){
    base_offset[4] -= suj_offset;
  }
  else if (event.getChar() == 'Y'){
    base_offset[5] -= suj_offset;
  }

  //the arm offset
  else if (event.getChar() == 'z'){
    arm_offset[0] += j_offset;
  }
  else if (event.getChar() == 'x'){
    arm_offset[1] += j_offset;
  }
  else if (event.getChar() == 'c'){
    arm_offset[2] += j_offset;
  }
  else if (event.getChar() == 'v'){
    arm_offset[3] += SCALE_FACTOR * j_offset;
  }
  else if (event.getChar() == 'b'){
    arm_offset[4] += SCALE_FACTOR * j_offset;
  }
  else if (event.getChar() == 'n'){
    arm_offset[5] += SCALE_FACTOR * j_offset;
  }
  else if (event.getChar() == 'm'){
    arm_offset[6] += SCALE_FACTOR * j_offset;
  }
  else if (event.getChar() == 'Z'){
    arm_offset[0] -= suj_offset;
  }
  else if (event.getChar() == 'X'){
    arm_offset[1] -= suj_offset;
  }
  else if (event.getChar() == 'C'){
    arm_offset[2] -= suj_offset;
  }
  else if (event.getChar() == 'V'){
    arm_offset[3] -= SCALE_FACTOR * j_offset;
  }
  else if (event.getChar() == 'B'){
    arm_offset[4] -= SCALE_FACTOR * j_offset;
  }
  else if (event.getChar() == 'N'){
    arm_offset[5] -= SCALE_FACTOR * j_offset;
  }
  else if (event.getChar() == 'M'){
    arm_offset[6] -= SCALE_FACTOR * j_offset;
  }

  console() << "Trackable " << model_idx << " base offset is : \n[";
  for (size_t i = 0; i < base_offset.size(); ++i){
    console() << base_offset[i];
    if (i != base_offset.size() - 1)  console() << ", ";
  }
  console() << "]\n";
  console() << "Trackable " << model_idx << " offset is : \n[";
  for (size_t i = 0; i < arm_offset.size(); ++i){
    console() << arm_offset[i];
    if (i != arm_offset.size() - 1)  console() << ", ";
  }
  console() << "]" << std::endl;

}

