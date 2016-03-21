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
#include <deque>

//#ifdef _DEBUG
//#undef _DEBUG
//#include <Python.h>
//#define _DEBUG
//#else
//#include <Python.h>
//#endif
//
//#define BOOST_PYTHON_STATIC_LIB 
//#include <boost/python.hpp>

#include "../include/config_reader.hpp"
#include "../include/vizApp.hpp"
#include "../include/resources.hpp"
#include "../include/fourier_descriptor.hpp"

using namespace viz;

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
        boost::filesystem::create_directories(output_dir);

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

      try{
        tracked_camera_.reset(new PoseGrabber(ConfigReader(root_dir + "/" + reader.get_element("tracked-camera")), output_dir_this_run)); //if there is no moveable camera then there won't be a tracked camera
      }
      catch (std::runtime_error){
        tracked_camera_.reset(new QuaternionPoseGrabber(ConfigReader(root_dir + "/" + reader.get_element("tracked-camera")), output_dir_this_run)); //if there is no moveable camera then there won't be a tracked camera
      }
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
    
    state.load_all = true;

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
 
  ci::app::setFrameRate(200);

  //Py_Initialize();

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

inline cv::Vec2i toOpenCV(const ci::Vec2i &v) { return cv::Vec2i(v[0], v[1]); }


std::vector<cv::Point> UpdateContour(const cv::Mat &image){

  std::vector<cv::Point> updated_contour;
  for (int r = 0; r < image.rows; ++r){
    for (int c = 0; c < image.cols; ++c){
      if (image.at<unsigned char>(r, c) == 255){
        updated_contour.push_back(cv::Point(c, r));
      }
    }
  }

  return updated_contour;
}


void DrawContour(const std::vector<cv::Point> &contour, cv::Mat &frame){

  frame.setTo(0);

  std::vector<std::vector<cv::Point> > t;
  t.push_back(contour);

  drawContours(frame, t, -1, cv::Scalar(255), 1, 8);

}

std::vector < str::EllipticalFourierDescriptor > GetFourierDescriptorsFromContour(const std::vector< cv::Point> &contour){

  static str::EllipticalFourierDescriptorBuilder builder;

  auto descriptors = builder.BuildFromContour(contour);

  return descriptors;

}

bool InPreviousPoints(const cv::Point &current_point, const std::deque<cv::Point> &contour){

  int j = 0;
  for (int i = contour.size() - 1; i > 0 && j < 4; --i, ++j){
    if (current_point == contour[i]) return true;
  }

  return false;

}

bool FindAdjacentPixel(const cv::Mat &image, const cv::Point &start_point, cv::Point &current_point, std::deque < cv::Point> &contour){

  cv::Point neighbour_point = current_point;

  while (true){
    
    //N
    cv::Point tmp_neighbour_point = neighbour_point; tmp_neighbour_point.y -= 1;
    if (cv::Rect(0, 0, image.cols, image.rows).contains(tmp_neighbour_point) && image.at<unsigned char>(tmp_neighbour_point) && InPreviousPoints(tmp_neighbour_point, contour)){
      neighbour_point = tmp_neighbour_point;
      contour.push_back(neighbour_point);
      int index = contour.size() - 1;
      if (FindAdjacentPixel(image, start_point, neighbour_point, contour)){
        break;
      }
      else{
        contour.erase(contour.begin() + index);
      }
    }

    //NE
    tmp_neighbour_point = neighbour_point; tmp_neighbour_point.y -= 1; tmp_neighbour_point.x += 1;
    if (cv::Rect(0, 0, image.cols, image.rows).contains(tmp_neighbour_point) && image.at<unsigned char>(tmp_neighbour_point) && InPreviousPoints(tmp_neighbour_point, contour)){
      neighbour_point = tmp_neighbour_point;
      if (FindAdjacentPixel(image, start_point, neighbour_point, contour)){
        contour.push_front(neighbour_point);
        break;
      }
    }

    //E
    tmp_neighbour_point = neighbour_point; tmp_neighbour_point.y += 1;
    if (cv::Rect(0, 0, image.cols, image.rows).contains(tmp_neighbour_point) && image.at<unsigned char>(tmp_neighbour_point) && InPreviousPoints(tmp_neighbour_point, contour)){
      neighbour_point = tmp_neighbour_point;
      if (FindAdjacentPixel(image, start_point, neighbour_point, contour)){
        contour.push_front(neighbour_point);
        break; 
      }
    }

    //SE
    tmp_neighbour_point = neighbour_point; tmp_neighbour_point.y += 1; tmp_neighbour_point.x += 1;
    if (cv::Rect(0, 0, image.cols, image.rows).contains(tmp_neighbour_point) && image.at<unsigned char>(tmp_neighbour_point) && InPreviousPoints(tmp_neighbour_point, contour)){
      neighbour_point = tmp_neighbour_point;
      if (FindAdjacentPixel(image, start_point, neighbour_point, contour)){
        contour.push_front(neighbour_point);
        break;
      }
    }

    //S
    tmp_neighbour_point = neighbour_point; tmp_neighbour_point.y += 1;
    if (cv::Rect(0, 0, image.cols, image.rows).contains(tmp_neighbour_point) && image.at<unsigned char>(tmp_neighbour_point) && InPreviousPoints(tmp_neighbour_point, contour)){
      neighbour_point = tmp_neighbour_point;
      if (FindAdjacentPixel(image, start_point, neighbour_point, contour)){
        contour.push_front(neighbour_point);
        break;
      }
    }

    //N
    tmp_neighbour_point = neighbour_point; tmp_neighbour_point.y -= 1;
    if (cv::Rect(0, 0, image.cols, image.rows).contains(tmp_neighbour_point) && image.at<unsigned char>(tmp_neighbour_point) && InPreviousPoints(tmp_neighbour_point, contour)){
      neighbour_point = tmp_neighbour_point;
      if (FindAdjacentPixel(image, start_point, neighbour_point, contour)){
        contour.push_front(neighbour_point);
        break;
      }
    }

    //N
    tmp_neighbour_point = neighbour_point; tmp_neighbour_point.y -= 1;
    if (cv::Rect(0, 0, image.cols, image.rows).contains(tmp_neighbour_point) && image.at<unsigned char>(tmp_neighbour_point) && InPreviousPoints(tmp_neighbour_point, contour)){
      neighbour_point = tmp_neighbour_point;
      if (FindAdjacentPixel(image, start_point, neighbour_point, contour)){
        contour.push_front(neighbour_point);
        break;
      }
    }

    //N
    tmp_neighbour_point = neighbour_point; tmp_neighbour_point.y -= 1;
    if (cv::Rect(0, 0, image.cols, image.rows).contains(tmp_neighbour_point) && image.at<unsigned char>(tmp_neighbour_point) && InPreviousPoints(tmp_neighbour_point, contour)){
      neighbour_point = tmp_neighbour_point;
      if (FindAdjacentPixel(image, start_point, neighbour_point, contour)){
        contour.push_front(neighbour_point);
        break;
      }
    }

    break;

  }
  
  if (neighbour_point == start_point) return false;



}

std::vector<cv::Point> CreateFreemanChain(const std::vector<cv::Point> &contour, size_t image_width, size_t image_height){

  cv::Mat image = cv::Mat::zeros(cv::Size(image_width, image_height), CV_8UC1);
  for (auto &px : contour){

    image.at<unsigned char>(px.y, px.x) = 1;

  }

  cv::Point start_point;
  for (size_t r = 0; r < image_height; ++r){
    for (size_t c = 0; c < image_width; ++c){

      if (image.at<unsigned char>(r, c) == 1){
        start_point = cv::Point(c, r);
        break;
      }

    }
  }

  std::vector<cv::Point> output_contour;
  bool add = false;
  for (auto &pix : contour){

    if (pix == start_point) add = true;
    if (add)
      output_contour.push_back(pix);

  }

  for (auto &pix : contour){

    if (pix == start_point) add = false;
    if (add)
      output_contour.push_back(pix);

  }

  return output_contour;

}

void vizApp::EvaluatePoseFromFrame(cv::Mat &frame){

  //static std::ifstream pose_file("C:/Users/max/libs/str-forest/data/no_6dof_data/d0/output/output_poses.txt");

  //float x_translation, y_translation, z_translation, x_rotation, y_rotation, z_rotation;
  //float wrist_angle, clasper_direction, clasper_angle;

  //pose_file >> x_translation >> y_translation >> z_translation >> x_rotation >> y_rotation >> z_rotation >> wrist_angle >> clasper_direction >> clasper_angle;

  /*
  ci::app::console() << "HEre" << std::endl;

  std::vector<cv::Point> largest_contour = GetContourFromFrame(frame);
  auto descriptors = GetFourierDescriptorsFromContour(largest_contour);

  std::vector<float> flattened_descriptor;
  for (auto &d : descriptors){
    for (int i = 0; i < 4; ++i)
      flattened_descriptor.push_back(d[i]);
  }

  //boost::python::exec("from sklearn.ensemble import RandomForestRegressor", main_namespace);
  static PyObject *file = PyFile_FromString("C:/Users/max/libs/str-forest/proto/forest_articulated_only.pkl", "rb");
  //static PyObject *module_name = PyString_FromString("pickle");

  static PyObject *sys = PyImport_ImportModule("sys");
  static PyObject *path = PyObject_GetAttrString(sys, "path");
  static bool first = true;
  if (first){
    PyList_Append(path, PyString_FromString("C:/Python27/Lib/"));
    first = false;
  }

  ci::app::console() << "HEre 2" << std::endl;
  static PyObject *module = PyImport_ImportModule("pickle");
  //PyObject *ptype, *pvalue, *ptraceback;
  //PyErr_Fetch(&ptype, &pvalue, &ptraceback);
  //std::string pStrErrorMessage = PyString_AsString(pvalue);
  
  static PyObject *function = PyObject_GetAttrString(module, "load");
  static PyObject *args = PyTuple_Pack(1, file);
  static PyObject *predictor = PyObject_CallObject(function, args);

  //convert flattened_desctiptor to python list
  PyObject* descriptor = PyList_New(flattened_descriptor.size());
  for (Py_ssize_t i = 0; i < flattened_descriptor.size(); i++) {
    PyList_SetItem(descriptor, i, PyFloat_FromDouble(flattened_descriptor[i]));
  }
  ci::app::console() << "HEre 3" << std::endl;
  PyObject *pose = PyObject_CallMethodObjArgs(predictor, PyString_FromString("predict"), descriptor, NULL);
  if (pose == NULL) {
    ci::app::console() << "Pose is null" << std::endl;
    PyObject *ptype, *pvalue, *ptraceback;
    PyErr_Fetch(&ptype, &pvalue, &ptraceback);
    std::string pStrErrorMessage = PyString_AsString(pvalue);
    ci::app::console() << pStrErrorMessage << std::endl;
  }
  ci::app::console() << "HEre 3a" << std::endl;
  PyObject *tolist = PyObject_GetAttrString(pose, "tolist");
  ci::app::console() << "HEre 3b" << std::endl;
  PyObject *listoflist = PyObject_CallObject(tolist, NULL);
  ci::app::console() << "HEre 3c" << std::endl;

  std::vector<float> pose_cpp;
  //convert pose back to c++ vector
  PyObject *list = PyList_GetItem(listoflist, 0);
  ci::app::console() << "HEre 3d" << std::endl;
  for (Py_ssize_t i = 0; i < PyList_Size(list); i++) {
    PyObject *p = PyList_GetItem(list, i);
    ci::app::console() << "HEre 3e" << std::endl;
    pose_cpp.push_back(PyFloat_AsDouble(p));
    ci::app::console() << "HEre 3f" << std::endl;
  }
  ci::app::console() << "HEre 4" << std::endl;

  */

  static std::ifstream ifs("C:/Users/max/libs/str-forest/data/no_6dof_data/d0/output.txt");



  boost::shared_ptr<SE3DaVinciPoseGrabber> v = boost::dynamic_pointer_cast<SE3DaVinciPoseGrabber>(trackables_[0]);

  /*
  float x_translation = pose_cpp[0];
  float y_translation = pose_cpp[1];
  float z_translation = pose_cpp[2];
  float x_rotation = pose_cpp[3];
  float y_rotation = pose_cpp[4];
  float z_rotation = pose_cpp[5];
  float wrist_angle = pose_cpp[6];
  float clasper_direction = pose_cpp[7];
  float clasper_angle = pose_cpp[8]; 
  */
  float x_translation;
  float y_translation;
  float z_translation;
  float x_rotation;
  float y_rotation;
  float z_rotation;    
  float wrist_angle;
  float clasper_direction;
  float clasper_angle;
  
  ifs >> x_translation >> y_translation >> z_translation >> x_rotation >> y_rotation >> z_rotation >> wrist_angle >> clasper_direction >> clasper_angle;

  ci::app::console() << "HEre 5" << std::endl;

  v->EditPose(ci::Vec3f(x_translation, y_translation, z_translation), ci::Vec3f(x_rotation, y_rotation, z_rotation), ci::Vec3f(wrist_angle, clasper_direction, clasper_angle));

  left_eye.Bind();
  gl::enableDepthRead();
  gl::enableDepthWrite();
  gl::pushMatrices();

  ci::app::console() << "HEre 6" << std::endl;
  camera_.setupLeftCamera(maya_cam_, getCameraPose()); //do viewport and set camera pose
  
  //shader_.bind();
  //shader_.uniform("tex0", 0);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  drawTargets();
  glDisable(GL_BLEND);

  //shader_.unbind();

  gl::popMatrices();

  camera_.unsetCameras(); //reset the viewport values
  left_eye.UnBind();

  ci::app::console() << "HEre 7" << std::endl;
}

std::vector<cv::Point> vizApp::GetContourFromFrame(cv::Mat &frame){

  boost::shared_ptr<SE3DaVinciPoseGrabber> v = boost::dynamic_pointer_cast<SE3DaVinciPoseGrabber>(trackables_[0]);
  std::array<ci::Vec2i, 4> rectangle;
  cv::Mat affine_transform;
  v->GetSubWindowCoordinates(camera_.GetLeftCamera(), rectangle, affine_transform);

  cv::Mat view_frame = frame.clone();

  cv::line(view_frame, toOpenCV(rectangle[0]), toOpenCV(rectangle[1]), cv::Scalar(255, 0, 0), 3);
  cv::line(view_frame, toOpenCV(rectangle[1]), toOpenCV(rectangle[2]), cv::Scalar(255, 0, 0), 3);
  cv::line(view_frame, toOpenCV(rectangle[2]), toOpenCV(rectangle[3]), cv::Scalar(255, 0, 0), 3);
  cv::line(view_frame, toOpenCV(rectangle[3]), toOpenCV(rectangle[0]), cv::Scalar(255, 0, 0), 3);

  const float width = std::sqrtf((rectangle[0].x - rectangle[1].x)*(rectangle[0].x - rectangle[1].x) + (rectangle[0].y - rectangle[1].y)*(rectangle[0].y - rectangle[1].y));
  const float height = std::sqrtf((rectangle[1].x - rectangle[2].x)*(rectangle[1].x - rectangle[2].x) + (rectangle[1].y - rectangle[2].y)*(rectangle[1].y - rectangle[2].y));

  std::vector<cv::Point2f> src_points;
  src_points.push_back(cv::Point2f(toOpenCV(rectangle[0])));
  src_points.push_back(cv::Point2f(toOpenCV(rectangle[1])));
  //src_points.push_back(cv::Point2f(toOpenCV(rectangle[2])));
  src_points.push_back(cv::Point2f(toOpenCV(rectangle[3])));

  std::vector<cv::Point2f> dst_points;
  dst_points.push_back(cv::Point2f(0, 0));
  dst_points.push_back(cv::Point2f(width, 0));
  dst_points.push_back(cv::Point2f(0, height));
  //dst_points.push_back(cv::Point2f(width, height));

  affine_transform = cv::getAffineTransform(src_points, dst_points);

  cv::Mat output_frame;
  cv::warpAffine(frame, output_frame, affine_transform, cv::Size(width, height));

  cv::Mat contour_image = cv::Mat::zeros(output_frame.size(), CV_8UC1);
  for (int r = 4; r < contour_image.rows - 4; ++r){
    for (int c = 4; c < contour_image.cols - 4; ++c){
      if (output_frame.at<cv::Vec4b>(r, c) != cv::Vec4b(0, 0, 255, 255)){
        contour_image.at<unsigned char>(r, c) = 255;
      }
    }
  }

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(contour_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  
  frame = view_frame;

  std::vector<cv::Point> largest;
  for (size_t i = 0; i < contours.size(); ++i){
    if (contours[i].size() > largest.size()) largest = contours[i];
  }

  return largest;

}

void vizApp::CreateContourFromFrame(cv::Mat &frame){

  boost::filesystem::create_directories(SubWindow::output_directory);
  //static std::ofstream contour_file(SubWindow::output_directory + "/contour.txt");
  static std::ofstream feature_file(SubWindow::output_directory + "/features.txt");

  std::vector<cv::Point> largest_contour = GetContourFromFrame(frame);

  if (largest_contour.size() < 10){
    feature_file << "-1";
  }
  else{

    //for (size_t i = 0; i < largest_contour.size(); ++i){

    //auto &pix = largest_contour[i];
    //contour_file << pix.x << ", " << pix.y;
    //if (i != (largest_contour.size() - 1)) contour_file << ", ";

    //}
    //contour_file << std::endl;

    auto descriptors = GetFourierDescriptorsFromContour(largest_contour);

    for (size_t i = 0; i < descriptors.size(); ++i){

      auto &descriptor = descriptors[i];
      feature_file << descriptor.vals_[0] << ", " << descriptor.vals_[1] << ", " << descriptor.vals_[2] << ", " << descriptor.vals_[3];
      if (i != (descriptors.size() - 1)) feature_file << ", ";

    }
  }
  feature_file << std::endl;


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
  cv::Mat f = left_eye.getFrame();
  
  ///** draw right eye **/
  //right_eye.BindAndClear();
  //drawRightEye();
  //right_eye.UnBind();
  //right_eye.Draw();

  ///** draw scene **/
  //scene_viewer.BindAndClear();
  //drawScene(left_texture_, right_texture_);
  //scene_viewer.UnBind();
  //scene_viewer.Draw();

  //trajectory_viewer.BindAndClear();
  //drawCameraTracker();
  //trajectory_viewer.UnBind();
  //trajectory_viewer.Draw();
  
  saveState();

  //dont actually use frame
  //EvaluatePoseFromFrame(f);

  cv::flip(f, f, 0);
  CreateContourFromFrame(f);
  cv::flip(f, f, 0);
  left_eye.ReplaceFrame(f);
  left_eye.Draw();

  state.load_one = false;
  state.save_one = false;

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

void vizApp::save2DTrack(){

  static std::vector<std::ofstream> files;
  static std::vector<float> scales;

  if (files.size() == 0){

    shaft_framebuffer = gl::Fbo(camera_image_width_, camera_image_height_);
    head_framebuffer = gl::Fbo(camera_image_width_, camera_image_height_);

    for (size_t i = 0; i < trackables_.size(); ++i){
      std::stringstream ss;
      if (!boost::filesystem::exists(SubWindow::output_directory)){
        boost::filesystem::create_directory(SubWindow::output_directory);
      }
      ss << SubWindow::output_directory << "/track_file" << i << ".txt";
      files.push_back(std::ofstream(ss.str(), 'w'));
      
    }

    

  }


  for (size_t i = 0; i < files.size(); ++i){

    gl::enableDepthRead();
    gl::enableDepthWrite();
    gl::pushMatrices();
    glDisable(GL_TEXTURE_2D);

    camera_.setupLeftCamera(maya_cam_, getCameraPose()); //do viewport and set camera pose
    boost::shared_ptr<DHDaVinciPoseGrabber> lnd = boost::dynamic_pointer_cast<DHDaVinciPoseGrabber>(trackables_[i]);
    ci::Matrix44f shaft_pose, head_pose, clasper_left_pose, clasper_right_pose;
    shaft_pose = lnd->GetPose();
    lnd->GetModelPose(head_pose, clasper_left_pose, clasper_right_pose);

    //shaft 
    shaft_framebuffer.bindFramebuffer();
    gl::clear();
    gl::color(1.0, 1.0, 1.0);
    ci::Vec3f origin(0, 0, 0);
    ci::Vec3f shaft_dir(0, 0, -5);
    gl::pushModelView();
    ci::gl::multModelView(shaft_pose);
    gl::drawLine(origin, shaft_dir);

    float current_scale = 1.0;

    if (scales.size() <= i){
      ci::Matrix44f pose = ci::gl::getModelView();
      scales.push_back(pose.getTranslate().z);
    }
    else{
      ci::Matrix44f pose = ci::gl::getModelView();
      current_scale = scales[i] / pose.getTranslate().z;
    }

    gl::popModelView();
    shaft_framebuffer.unbindFramebuffer();
    glFinish();

    //head
    head_framebuffer.bindFramebuffer();
    gl::clear();
    gl::color(1.0, 1.0, 1.0);
    gl::pushModelView();
    //ci::gl::multModelView(shaft_pose);
    gl::multModelView(head_pose);
    gl::drawSphere(ci::Vec3f(0, 0, 0), 3);
    head_framebuffer.unbindFramebuffer();
    gl::popModelView();
    glFinish();

    cv::Mat shaft_axis_ = toOcv(shaft_framebuffer.getTexture());
    cv::Mat shaft_axis;
    cv::flip(shaft_axis_, shaft_axis, 0);
    cv::Mat head_ = toOcv(head_framebuffer.getTexture());
    cv::Mat head;
    cv::flip(head_, head, 0);

    ci::Vec2f center_of_head = GetCOM(head);
    ci::Vec2f start_of_shaft = GetEnd(shaft_axis, i == 0);


    ci::Vec2f unit_vector_along_shaft(-1, -1);
    if (start_of_shaft != ci::Vec2f(-1, -1) && center_of_head != ci::Vec2f(-1, -1)){
      unit_vector_along_shaft = center_of_head - start_of_shaft;
      unit_vector_along_shaft.normalize();
    }

    gl::popMatrices();
    camera_.unsetCameras(); //reset the viewport values
    glEnable(GL_TEXTURE_2D);

    std::stringstream to_write;
    to_write << center_of_head[0] << ", " << center_of_head[1] << ", " << unit_vector_along_shaft[0] << ", " << unit_vector_along_shaft[1] << ", " << current_scale;
    files[i] << to_write.str() << std::endl;

  }

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

  //Py_Finalize();

  AppNative::shutdown();
  AppNative::quit();

}

void vizApp::drawTargets(){

  for (size_t i = 0; i < trackables_.size(); ++i){
    trackables_[i]->Draw();
  }

}

ci::Matrix44f vizApp::getCameraPose(){

  if (tracked_camera_){
    return tracked_camera_->GetPose();
  }
  else if (moveable_camera_){
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
    //trackables_.push_back(boost::shared_ptr<BasePoseGrabber>(new DHDaVinciPoseGrabber(reader, output_dir)));
    //return;
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
    trackables_.push_back(boost::shared_ptr<QuaternionPoseGrabber>(new QuaternionPoseGrabber(reader, output_dir)));
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

  else if (event.getChar() == 'c'){
  
    for (auto &tracked_model : trackables_){


      boost::shared_ptr<DHDaVinciPoseGrabber> h = boost::dynamic_pointer_cast<DHDaVinciPoseGrabber>(tracked_model);

      h->SetOffsetsToNull();

    }

  }
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
