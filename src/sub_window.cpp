#include "../include/sub_window.hpp"
#include "../include/vizApp.hpp"

using namespace viz;

std::string SubWindow::output_directory;

void SubWindow::Init(const std::string &name, int start_x, int start_y, int eye_width, int eye_height, bool can_save){

  Init(name, start_x, start_y, eye_width, eye_height, eye_width, eye_height, can_save);

}

void SubWindow::Init(const std::string &name, int start_x, int start_y, int eye_width, int eye_height, int draw_width, int draw_height, bool can_save){

  name_ = name; 
  can_save_ = can_save;

  window_coords_ = ci::Rectf(start_x, start_y, start_x + draw_width, start_y + draw_height);
  ci::gl::Fbo::Format f;
  f.enableMipmapping();
  framebuffer_.reset(new ci::gl::Fbo(eye_width, eye_height, f));
  framebuffer_->getTexture().setFlipped(true);
  texture_ = ci::gl::Texture(eye_width, eye_height);

  if (can_save_){
    save_params_ = ci::params::InterfaceGl::create(ci::app::getWindow(), "Save Window", ci::app::toPixels(ci::Vec2i(100, 100)));
    save_params_->addButton("Save contents to file", std::bind(&SubWindow::InitSavingWindow, this));
    save_params_->show();
  }

  vizApp::AddSubWindow(this);

}

void SubWindow::Bind() {

  framebuffer_->bindFramebuffer();

}

void SubWindow::BindAndClear(){

  Bind();
  ci::gl::clear(ci::Color(0, 0, 0));

}

void SubWindow::UnBind() {
  framebuffer_->unbindFramebuffer();
}

bool SubWindow::CanSave() { 

  return can_save_; 

}


bool SubWindow::IsSaving() const{
  
  return writer_.isOpened();

}

void SubWindow::WriteFrameToFile(){

  cv::Mat window = toOcv(framebuffer_->getTexture());
  writer_.write(window);

}

void SubWindow::Draw(ci::params::InterfaceGlRef params){

  Draw(params, GetRectWithBuffer().getUpperLeft(), ci::Vec2i(Width(), Height()));

}

void SubWindow::Draw(ci::params::InterfaceGlRef params, ci::Vec2i tl, ci::Vec2i size){

  if (!params->isVisible()) return;

  std::stringstream position;
  position << "position='" << tl[0] << " " << tl[1] << "'";
  params->setOptions("", position.str());
  std::stringstream ssize;
  ssize << "size='" << size[0] << " " << size[1] << "'";
  params->setOptions("", ssize.str());
  params->draw();

}

void SubWindow::Draw(){

  //cv::Mat i = toOcv(framebuffer_->getTexture());
  //cv::Mat j;
  //cv::resize(i, j, cv::Size(window_coords_.getWidth(), window_coords_.getHeight()));
  //ci::gl::Texture t = fromOcv(j);
  
  ci::Rectf window_with_buffer = GetRectWithBuffer();

  ci::gl::draw(framebuffer_->getTexture(), window_with_buffer);// ci::Rectf(window_coords_.x1, window_coords_.y2, window_coords_.x2, window_coords_.y1));

  if (can_save_){
    Draw(save_params_, window_with_buffer.getUpperLeft() + ci::Vec2i(10, 10), ci::Vec2i(200, 50));
  }

}

void SubWindow::InitSavingWindow(){

  std::string &save_dir = output_directory;
  if (!boost::filesystem::exists(save_dir)) return;

  std::string name = name_;
  std::replace(name.begin(), name.end(), ' ', '_');
  std::string filepath = save_dir + "/" + name + ".avi";

  writer_.open(filepath, CV_FOURCC('M', 'J', 'P', 'G'), 25, cv::Size(texture_.getWidth(), texture_.getHeight()));

}