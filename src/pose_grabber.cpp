#include "../include/pose_grabber.hpp"
#include <cinder/Quaternion.h>

using namespace viz;

void BasePoseGrabber::convertFromBouguetPose(const ci::Matrix44f &in_pose, ci::Matrix44f &out_pose){

  out_pose.setToIdentity();

  ci::Vec3f translation = in_pose.getTranslate().xyz();
  translation[1] *= -1;
  translation[2] *= -1;
  out_pose.translate(translation);

  ci::Matrix33f flip;
  flip.setToIdentity();
  flip.at(1, 1) *= -1;
  flip.at(2, 2) *= -1;
  ci::Matrix33f in_gl_coords = flip * in_pose.subMatrix33(0, 0);
  ci::Quatf q(in_gl_coords);
  out_pose.rotate(q.getAxis(), q.getAngle());

}

PoseGrabber::PoseGrabber(const std::string &filename){

  ifs_.open(filename);
  if (!ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + filename);
  }
  
  ifs_.exceptions(std::ifstream::eofbit);

}

DaVinciPoseGrabber::DaVinciPoseGrabber(const std::string &in_suj_file, const std::string &in_j_file){

  suj_ifs_.open(in_suj_file);
  if (!suj_ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + in_suj_file);
  }

  suj_ifs_.exceptions(std::ifstream::eofbit);

  j_ifs_.open(in_j_file);
  if (!j_ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + in_j_file);
  }

  j_ifs_.exceptions(std::ifstream::eofbit);

}

Pose PoseGrabber::getNextPose(){

  ci::Matrix44f next_pose;
  next_pose.setToIdentity();

  try{
    for (int i = 0; i < 4; ++i){
      for (int j = 0; j < 4; ++j){
        float val;
        ifs_ >> val;
        next_pose.at(i, j) = val;
      }
    }
  }
  catch (std::ofstream::failure e){
    next_pose.setToIdentity();
  }

  return next_pose;

}
