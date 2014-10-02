#include "../include/trackable.hpp"
#include "davinci.hpp"

using namespace viz;

void Trackable::setupPoseGrabber(const std::string &filename){
  pose_grabber_.reset(new PoseGrabber(filename));
}

void Trackable::setupArticulatedPoseGrabber(const std::string &filename){
  pose_grabber_.reset(new ArticulatedPoseGrabber(filename));
}

void Trackable::setupPoseGrabber(const std::string &filename, const std::string &dh_out_file, const std::string &se3_out_file){

  dh_and_se3_ofs_.open(dh_out_file);
  se3_from_cam_ofs_.open(se3_out_file);

  if (!dh_and_se3_ofs_.is_open() || !se3_from_cam_ofs_.is_open()){
    throw std::runtime_error("Error, cannot open file!");
  }

  setupPoseGrabber(filename);
  
}

void Trackable::setupModel(boost::shared_ptr<ttrk::Model> model){
  trackable_ = model;
}

void Trackable::setupDaVinciModel(const std::string &filename){

  trackable_.reset(new ttrk::IntuitiveSurgicalLND(filename));

}

void Trackable::setupDaVinciPoseGrabber(const std::string &suj_file, const std::string &j_file, const std::string &se3_and_dh_out_file, const std::string &suj_dh_out_file, const std::string &j_dh_out_file, const std::string &se3_from_cam_out_file, const std::string &se3_from_world_out_file, davinci::DaVinciJoint joint_type){

  if (se3_and_dh_out_file != ""){
    dh_and_se3_ofs_.open(se3_and_dh_out_file);
    if (!dh_and_se3_ofs_.is_open()){
      throw std::runtime_error("");
    }
  }
  if (suj_dh_out_file != ""){
    suj_dh_ofs_.open(suj_dh_out_file);
    if (!suj_dh_ofs_.is_open()){
      throw std::runtime_error("");
    }
  }

  if (j_dh_out_file != ""){
    j_dh_ofs_.open(j_dh_out_file);
    if (!j_dh_ofs_.is_open()){
      throw std::runtime_error("");
    }
  }
  if (se3_from_cam_out_file != ""){
    se3_from_cam_ofs_.open(se3_from_cam_out_file);
    if (!se3_from_cam_ofs_.is_open()){
      throw std::runtime_error("");
    }
  }
  if (se3_from_world_out_file != ""){
    se3_from_world_ofs_.open(se3_from_world_out_file);
    if (!se3_from_world_ofs_.is_open()){
      throw std::runtime_error("");
    }
  }
  
  pose_grabber_.reset(new DaVinciPoseGrabber(suj_file, j_file, joint_type));

}

void Trackable::setupDaVinciPoseGrabber(const std::string &suj_file, const std::string &j_file, const std::string &se3_and_dh_out_file, const std::string &suj_dh_out_file, const std::string &j_dh_out_file, const std::string &se3_from_cam_out_file, const std::string &se3_from_world_out_file, const std::string &da_vinci_config, davinci::DaVinciJoint joint_type){


  setupDaVinciPoseGrabber(suj_file, j_file, se3_and_dh_out_file, suj_dh_out_file, j_dh_out_file, se3_from_cam_out_file, se3_from_world_out_file, joint_type);

  trackable_.reset(new ttrk::IntuitiveSurgicalLND(da_vinci_config));

}