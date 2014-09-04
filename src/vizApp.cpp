#include "../include/vizApp.hpp"
#include "../include/resources.hpp"
#include <CinderOpenCV.h>

using namespace viz;

const int WINDOW_WIDTH = 736;
const int WINDOW_HEIGHT = 288;

GLfloat no_mat[] = { 0.0, 0.0, 0.0, 1.0 };
ci::ColorA cNoMat = ci::ColorA(0.0f, 0.0f, 0.0f, 1.0f);

GLfloat mat_ambient[] = { 0.6, 0.3, 0.4, 1.0 };
ci::ColorA cAmbient = ci::ColorA(0.6f, 0.3f, 0.4f, 1.0f);

GLfloat mat_diffuse[] = { 0.3, 0.5, 0.8, 1.0 };
ci::ColorA cDiffuse = ci::ColorA(0.3f, 0.5f, 0.8f, 1.0f);

GLfloat mat_specular[] = { 0.508273, 0.508273, 0.508273, 1.0 };
ci::ColorA cSpecular = ci::ColorA(1.0f, 1.0f, 1.0f, 1.0f);

GLfloat mat_emission[] = { 0.0, 0.1, 0.3, 0.0 };
ci::ColorA cEmission = ci::ColorA(0.0f, 0.1f, 0.3f, 0.0f);

GLfloat mat_shininess[] = { 0.4 };
GLfloat no_shininess[] = { 0.0 };

void vizApp::setup(){

  const std::string root_dir("../config/seq1/");
  if (!boost::filesystem::is_directory(root_dir))
    throw std::runtime_error("Error, cannot file config dir!");

  shader_ = gl::GlslProg(loadResource(RES_SHADER_VERT), loadResource(RES_SHADER_FRAG));

  const std::string left_input_video(root_dir + "/" + "left.avi");
  const std::string right_input_video(root_dir + "/" + "right.avi");
  const std::string output_video(root_dir + "/" + "g.avi");
  const std::string camera_suj_file(root_dir + "/dv/" +"cam_suj.txt");
  const std::string camera_j_file(root_dir + "/dv/" + "cam_j.txt");
  const std::string psm1_dv_suj_file(root_dir + "/dv/" +"psm1_suj.txt");
  const std::string psm1_dv_j_file(root_dir + "/dv/" + "psm1_j.txt");
  const std::string psm2_dv_suj_file(root_dir + "/dv/" + "psm2_suj.txt");
  const std::string psm2_dv_j_file(root_dir + "/dv/" + "psm2_j.txt");
  const std::string da_vinci_config_file(root_dir + "/dv/" +"model/model.json");

  handler_.reset(new ttrk::StereoVideoHandler(left_input_video, right_input_video, output_video));
  
  camera_pg_.reset(new DaVinciPoseGrabber(camera_suj_file, camera_j_file, davinci::ECM));
  camera_.Setup(root_dir + "/camera_config.xml", WINDOW_WIDTH, WINDOW_HEIGHT, 1, 1000);
    
  //moving_objects_pg_.push_back( std::make_pair<>(boost::shared_ptr<Trackable>(new Trackable()),std::vector<ci::Matrix44f>()));
  //moving_objects_pg_.back().first->setupDaVinciPoseGrabber(psm2_dv_suj_file, psm2_dv_j_file, da_vinci_config_file, davinci::PSM2);
  moving_objects_pg_.push_back(boost::make_tuple<>(boost::shared_ptr<Trackable>(new Trackable()), std::vector<ci::Matrix44f>(), boost::shared_ptr<std::vector<double> >()));
  moving_objects_pg_.back().get<0>()->setupDaVinciPoseGrabber(psm1_dv_suj_file, psm1_dv_j_file, da_vinci_config_file, davinci::PSM1);
  moving_objects_pg_.back().get<0>()->getDVPoseGrabber()->setupOffsets(7);
  //moving_objects_pg_.back().get<0>()->setupPoseGrabber(root_dir + "/dv/grid.txt");

  setWindowSize(2 * WINDOW_WIDTH, WINDOW_HEIGHT);
  framebuffer_ = gl::Fbo(WINDOW_WIDTH, WINDOW_HEIGHT);
  load_next_image_ = true;

  draw2 = true;
  draw3 = true;

}

void vizApp::update(){

  camera_pose_ = camera_pg_->getPose(load_next_image_).poses_[0];


    for (std::size_t i = 0; i < moving_objects_pg_.size(); ++i){
      auto &mop = moving_objects_pg_[i];
      mop.get<1>() = mop.get<0>()->getDVPoseGrabber()->getPose(load_next_image_).poses_;
    }


  if (load_next_image_){

    cv::Mat stereo_image = handler_->GetNewFrame();

    cv::Mat left_frame = stereo_image(cv::Rect(0, 0, stereo_image.cols / 2, stereo_image.rows));
    cv::Mat right_frame = stereo_image(cv::Rect(stereo_image.cols / 2, 0, stereo_image.cols / 2, stereo_image.rows));

    left_texture_ = fromOcv(left_frame);
    right_texture_ = fromOcv(right_frame);
    load_next_image_ = false;

  }

}

void vizApp::draw(){
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) ); 

  framebuffer_.bindFramebuffer();
  drawEye(left_texture_, true);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf(0.0, (float)framebuffer_.getHeight(), (float)framebuffer_.getWidth(), 0.0));
  saveFrame(framebuffer_.getTexture(), true);

  framebuffer_.bindFramebuffer();
  drawEye(right_texture_, false);
  framebuffer_.unbindFramebuffer();
  gl::draw(framebuffer_.getTexture(), ci::Rectf((float)framebuffer_.getWidth(), (float)framebuffer_.getHeight(), 2.0 * framebuffer_.getWidth(), 0.0));
  saveFrame(framebuffer_.getTexture(), false);
  

}

void vizApp::saveFrame(gl::Texture texture, bool isLeft){
  
  cv::Mat frame = toOcv(texture);

  if (isLeft){
    write_left_ << frame;



  }
  else{
    write_right_ << frame;
  }

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

    const std::vector<ci::Matrix44f> &pose = mop.get<1>();

    auto meshes_textures_transforms = mop.get<0>()->getDaVinciTrackable()->GetRenderableMeshes();

    for (std::size_t i = 0; i < meshes_textures_transforms.size(); ++i){

      //shader_.bind();
      //shader_.uniform("NumEnabledLights", 1);        

      gl::pushModelView();
      const auto &mesh = meshes_textures_transforms[i].get<0>();
      const auto &texture = meshes_textures_transforms[i].get<1>();
      //const auto &transform = mesh_tex_trans.get<2>();
     
      gl::multModelView(pose[i]); //multiply by the pose of this tracked object

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
  //drawGrid();

  light.disable();

  gl::popMatrices(); 

  camera_.unsetCameras(); //reset the viewport values

}


void vizApp::drawGrid(float size, float step){
  
  const ci::Matrix44f &transform = moving_objects_pg_[0].get<1>()[0];

  gl::multModelView(transform);

  gl::color(Colorf(0.2f, 0.2f, 0.2f));

  for (float i = 0; i <= size; i += step){
    gl::drawLine(ci::Vec3f(0.0f, i, 0.0f), ci::Vec3f(size, i, 0.0f));
    gl::drawLine(ci::Vec3f(i, 0.0f, 0.0f), ci::Vec3f(i, size, 0.0f));
  }

  gl::color(1.0, 0.0, 0.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(0, 0, 0));
  gl::color(0.0, 1.0, 0.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(0, 3, 0));
  gl::color(0.0, 0.0, 1.0);
  gl::drawVector(ci::Vec3f(0, 0, 0), ci::Vec3f(0, 0, 3));


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


  boost::shared_ptr< std::vector< double > > offsets = moving_objects_pg_.back().get<0>()->getDVPoseGrabber()->getOffsets();

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
