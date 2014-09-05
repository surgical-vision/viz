#include "../include/trackable.hpp"
#include "davinci.hpp"

using namespace viz;

void Trackable::setupPoseGrabber(const std::string &filename, const std::string &dh_out_file, const std::string &se3_out_file){

  dh_ofs_.open(dh_out_file);
  se3_ofs_.open(se3_out_file);

  if (!dh_ofs_.is_open() || !se3_ofs_.is_open()){
    throw std::runtime_error("Error, cannot open file!");
  }

  pose_grabber_.reset(new PoseGrabber(filename));

}

void Trackable::setupDaVinciPoseGrabber(const std::string &suj_file, const std::string &j_file, const std::string &dh_out_file, const std::string &se3_out_file, const std::string &da_vinci_config, davinci::DaVinciJoint joint_type){


  dh_ofs_.open(dh_out_file);
  se3_ofs_.open(se3_out_file);

  if (!dh_ofs_.is_open() || !se3_ofs_.is_open()){
    throw std::runtime_error("Error, cannot open file!");
  }

  pose_grabber_.reset(new DaVinciPoseGrabber(suj_file, j_file, joint_type));
  trackable_.reset(new ttrk::IntuitiveSurgicalLND(da_vinci_config));

}