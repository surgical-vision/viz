#include "../include/trackable.hpp"

using namespace viz;

void Trackable::setupPoseGrabber(const std::string &filename){

  pose_grabber_.reset(new PoseGrabber(filename));

}

void Trackable::setupDaVinciPoseGrabber(const std::string &suj_file, const std::string &j_file, const std::string &da_vinci_config){

  pose_grabber_.reset(new DaVinciPoseGrabber(suj_file, j_file));
  trackable_.reset(new ttrk::IntuitiveSurgicalLND(da_vinci_config));

}