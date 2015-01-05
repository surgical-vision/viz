#include <boost/filesystem.hpp>
#include <cinder/ImageIo.h>
#include <cinder/app/App.h>

#include "../include/model.hpp"

using namespace viz;

ci::JsonTree BaseModel::OpenFile(const std::string &datafile_path) const {

  boost::filesystem::path p(datafile_path);

  if (p.extension().string() == ".json"){

    try{

      ci::JsonTree loader(ci::loadFile(datafile_path));
            
      return loader;

    }
    catch (ci::Exception &e){

      if (!boost::filesystem::exists(datafile_path))
        std::cout << "Error, cannot find file : " << datafile_path << std::endl;

      std::cout << e.what() << "\n";
      std::cout.flush();

    }
  }
  else{

    throw std::runtime_error("Error, unsupported file type");
  }

}

void BaseModel::InternalDraw(const RenderData &rd) const {

  ci::gl::pushModelView();

  ci::gl::multModelView(rd.transform_);

  glEnable(GL_COLOR_MATERIAL); //cinder uses colors rather than materials which are ignore by lighting unless you do this call.
  
  ci::gl::draw(rd.vbo_);

  glDisable(GL_COLOR_MATERIAL);

  ci::gl::popModelView();

}

void BaseModel::LoadComponent(const ci::JsonTree &tree, BaseModel::RenderData &target, const std::string &root_dir){

  boost::filesystem::path obj_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["obj-file"].getValue<std::string>());
  if (!boost::filesystem::exists(obj_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  boost::filesystem::path mat_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["mtl-file"].getValue<std::string>());
  if (!boost::filesystem::exists(mat_file)) throw(std::runtime_error("Error, the file doesn't exist!\n"));

  boost::filesystem::path tex_file = boost::filesystem::path(root_dir) / boost::filesystem::path(tree["texture"].getValue<std::string>());
  bool has_texture = false;
  //if (!boost::filesystem::exists(tex_file)) throw(std::runtime_error("Error, the file doens't exist!\n"));
  if (has_texture = boost::filesystem::exists(tex_file)){
    ci::gl::Texture::Format format;
    format.enableMipmapping(true);
    target.texture_ = ci::gl::Texture(ci::loadImage((tex_file.string())), format);
  }

  ci::ObjLoader loader(ci::loadFile(obj_file.string()), ci::loadFile(mat_file.string()));
  loader.load(&target.model_, true, has_texture, true);
  target.vbo_ = ci::gl::VboMesh(target.model_);

}

void Model::Draw() const {

  InternalDraw(body_);

}

void Model::LoadData(const std::string &datafile_path){

  ci::JsonTree tree = OpenFile(datafile_path);

  LoadComponent(tree, body_, boost::filesystem::path(datafile_path).parent_path().string());

}

std::vector<ci::Matrix44f> Model::GetTransformSet() const{
  return std::vector<ci::Matrix44f>({ body_.transform_ });
}

void Model::SetTransformSet(const std::vector<ci::Matrix44f> &transforms){
  assert(transforms.size() == 0);
  body_.transform_ = transforms[0];
}

void DaVinciInstrument::Draw() const {

  InternalDraw(shaft_);
  InternalDraw(head_);
  InternalDraw(clasper1_);
  InternalDraw(clasper2_);

}

void DaVinciInstrument::LoadData(const std::string &datafile_path){
  
  ci::JsonTree tree = OpenFile(datafile_path);

  LoadComponent(tree.getChild("shaft"), shaft_, boost::filesystem::path(datafile_path).parent_path().string());
  LoadComponent(tree.getChild("head"), head_, boost::filesystem::path(datafile_path).parent_path().string());
  LoadComponent(tree.getChild("clasper1"), clasper1_ , boost::filesystem::path(datafile_path).parent_path().string());
  LoadComponent(tree.getChild("clasper2"), clasper2_, boost::filesystem::path(datafile_path).parent_path().string());

}

std::vector<ci::Matrix44f> DaVinciInstrument::GetTransformSet() const{
  return std::vector<ci::Matrix44f>({ shaft_.transform_, head_.transform_, clasper1_.transform_, clasper2_.transform_});
}

void DaVinciInstrument::SetTransformSet(const std::vector<ci::Matrix44f> &transforms){
  
  assert(transforms.size() == 4);
  shaft_.transform_ = transforms[0];
  head_.transform_ = transforms[1];
  clasper1_.transform_ = transforms[2];
  clasper2_.transform_ = transforms[3];

}