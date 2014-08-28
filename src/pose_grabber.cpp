#include "../include/pose_grabber.hpp"
#include <cinder/Quaternion.h>
#include <cinder/app/App.h>
#include "api_stream.h"
#include "snippets.hpp"

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

  out_pose.invert(); //bouguet poses (from calibration) are grid poses so invert to get camera poses

}

PoseGrabber::PoseGrabber(const std::string &filename){

  ifs_.open(filename);
  if (!ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + filename);
  }
  
  ifs_.exceptions(std::ifstream::eofbit);

}

DaVinciPoseGrabber::DaVinciPoseGrabber(const std::string &in_suj_file, const std::string &in_j_file, const davinci::DaVinciJoint joint_type){

  target_joint_ = joint_type;

  switch (target_joint_){

  case davinci::DaVinciJoint::PSM1:
    num_suj_joints_ = chain_.mSUJ1OriginSUJ1Tip.size();
    num_j_joints_ = chain_.mPSM1OriginPSM1Tip.size();
    break;
  case davinci::DaVinciJoint::PSM2:
    num_suj_joints_ = chain_.mSUJ2OriginSUJ2Tip.size();
    num_j_joints_ = chain_.mPSM2OriginPSM2Tip.size();
    break;
  case davinci::DaVinciJoint::ECM:
    num_suj_joints_ = chain_.mSUJ3OriginSUJ3Tip.size();
    num_j_joints_ = 4;//chain_.mECM1OriginECM1Tip.size(); //alhtough this value is 4 in the file in pratt's code it is 7
    break;

  }

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

  ci::Matrix44f gl_next_pose;
  convertFromBouguetPose(next_pose, gl_next_pose);

  return gl_next_pose;

}

Pose DaVinciPoseGrabber::getNextPose(){

  std::vector<double> suj_joints, j_joints;
  ReadDHFromFiles(suj_joints, j_joints);

  Pose suj_frames, j_frames;

  if (target_joint_ == davinci::ECM){
    API_ECM ecm;

    for (std::size_t i = 0; i < suj_joints.size(); ++i){
      ecm.sj_joint_angles[i] = suj_joints[i];
    }

    for (std::size_t i = 0; i < j_joints.size(); ++i){
      ecm.jnt_pos[i] = j_joints[i];
    }

    GLdouble ecm_transform[16];
    buildKinematicChainECM1(chain_, ecm, ecm_transform, suj_frames, j_frames);

  }

  else if (target_joint_ == davinci::PSM1 || target_joint_ == davinci::PSM2){
    
    API_PSM psm;

    for (std::size_t i = 0; i < suj_joints.size(); ++i){
      psm.sj_joint_angles[i] = suj_joints[i];
    }

    for (std::size_t i = 0; i < j_joints.size(); ++i){
      psm.jnt_pos[i] = j_joints[i];
    }

    GLdouble psm_transform[16];
    if (target_joint_ == davinci::PSM1)
      buildKinematicChainPSM1(chain_, psm, psm_transform, suj_frames, j_frames);
    else if (target_joint_ == davinci::PSM2)
      buildKinematicChainPSM2(chain_, psm, psm_transform, suj_frames, j_frames);
  }

  //translate then rotate gives the same results

  return j_frames;

}

void DaVinciPoseGrabber::convertFromDaVinciPose(const ci::Matrix44f &in_pose, ci::Matrix44f &out_pose){

  assert(0);

  out_pose.setToIdentity();
  
  ci::Matrix33f flip;
  flip.setToIdentity();
  flip.at(0, 0) *= -1;
  flip.at(2, 2) *= -1;
  ci::Quatf q(flip);
  out_pose.rotate(q.getAxis(), q.getAngle());

  ci::Vec3f translation = in_pose.getTranslate().xyz();
  out_pose.translate(translation);
  
  ci::Matrix33f in_gl_coords = in_pose.subMatrix33(0, 0);
  ci::Quatf q2(in_gl_coords);
  out_pose.rotate(q2.getAxis(), q2.getAngle());

  //ci::Vec3f translation = in_pose.getTranslate().xyz();
  //translation[0] *= -1;
  //translation[2] *= -1;
  //out_pose.translate(translation);

  //ci::Matrix33f flip;
  //flip.setToIdentity();
  //flip.at(0, 0) *= -1;
  //flip.at(2, 2) *= -1;
  //ci::Matrix33f in_gl_coords = flip * in_pose.subMatrix33(0, 0);
  //ci::Quatf q(in_gl_coords);
  //out_pose.rotate(q.getAxis(), q.getAngle());

}

void DaVinciPoseGrabber::ReadDHFromFiles(std::vector<double> &psm_suj_joints, std::vector<double> &psm_joints){

  try{
    for (int i = 0; i < num_j_joints_; ++i){
      double x;
      j_ifs_ >> x;
      psm_joints.push_back(x);
    }

    for (int i = 0; i < num_suj_joints_; ++i){
      double x;
      suj_ifs_ >> x;
      psm_suj_joints.push_back(x);
    }

  }
  catch (std::ifstream::failure){
    exit(0);
  }

}