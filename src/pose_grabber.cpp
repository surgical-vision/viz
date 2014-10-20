#include <cinder/Quaternion.h>
#include <cinder/app/App.h>

#include "api_stream.h"
#include "snippets.hpp"
#include "../include/pose_grabber.hpp"


using namespace viz;

inline void clean_string(std::string &str, const std::vector<char> &to_remove){
  for (auto &ch : to_remove){
    str.erase(std::remove(str.begin(), str.end(), ch), str.end());
  }
}

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

PoseGrabber::PoseGrabber(const ConfigReader &reader){

  model_.LoadData(reader.get_element("model-file"));
  ifs_.open(reader.get_element("pose-file"));

  if (!ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + reader.get_element("pose-file"));
  }

  ifs_.exceptions(std::ifstream::eofbit);

}

void PoseGrabber::LoadPose(){

  ci::Matrix44f next_pose;

  next_pose.setToIdentity();
  do_draw_ = false; //set to true only if we read a 'good' pose

  try{
    std::string line;
    int row = 0;
    while (1)
    {
      std::getline(ifs_, line);
      if (row == 4) break;
      if (line[0] == '#' || line.length() < 1) continue;
      std::stringstream ss(line);
      for (int col = 0; col < 4; ++col){
        float val;
        ss >> val;
        next_pose.at(row, col) = val;
      }
      row++;
    }
    do_draw_ = true;
  }
  catch (std::ofstream::failure e){
    next_pose.setToIdentity();
    do_draw_ = false;
  }

  std::vector<ci::Matrix44f> ret({ next_pose });
  model_.SetTransformSet(ret);

}

BaseDaVinciPoseGrabber::BaseDaVinciPoseGrabber(const ConfigReader &reader){
  
  model_.LoadData(reader.get_element("model-file"));

}

void BaseDaVinciPoseGrabber::convertFromDaVinciPose(const ci::Matrix44f &in_pose, ci::Matrix44f &out_pose){


  out_pose.setToIdentity();

  ci::Matrix44f flip;
  flip.setToIdentity();
  flip.at(1, 1) *= -1;
  flip.at(2, 2) *= -1;

  flip.invert();

  out_pose = in_pose * flip;

}

DHDaVinciPoseGrabber::DHDaVinciPoseGrabber(const ConfigReader &reader) : BaseDaVinciPoseGrabber(reader) {

  if (reader.get_element("joint") == "PSM1")
    target_joint_ = davinci::DaVinciJoint::PSM1;
  else if (reader.get_element("joint") == "PSM2")
    target_joint_ = davinci::DaVinciJoint::PSM2;
  else if (reader.get_element("joint") == "ECM")
    target_joint_ = davinci::DaVinciJoint::ECM;
  else
    throw std::runtime_error("Error, bad joint");

  switch (target_joint_){

  case davinci::DaVinciJoint::PSM1:
    num_base_joints_ = chain_.mSUJ1OriginSUJ1Tip.size();
    num_arm_joints_ = chain_.mPSM1OriginPSM1Tip.size();
    break;
  case davinci::DaVinciJoint::PSM2:
    num_base_joints_ = chain_.mSUJ2OriginSUJ2Tip.size();
    num_arm_joints_ = chain_.mPSM2OriginPSM2Tip.size();
    break;
  case davinci::DaVinciJoint::ECM:
    num_base_joints_ = chain_.mSUJ3OriginSUJ3Tip.size();
    num_arm_joints_ = 4;//chain_.mECM1OriginECM1Tip.size(); //alhtough this value is 4 in the file in pratt's code it is 7
    break;
  }

  base_ifs_.open(reader.get_element("base-joint-file"));
  if (!base_ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + reader.get_element("base-joint-file"));
  }

  base_ifs_.exceptions(std::ifstream::eofbit);

  arm_ifs_.open(reader.get_element("arm-joint-file"));
  if (!arm_ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + reader.get_element("arm-joint-file"));
  }

  arm_ifs_.exceptions(std::ifstream::eofbit);

  model_.LoadData(reader.get_element("model-file"));

}

void DHDaVinciPoseGrabber::LoadPose(){

  std::vector<double> base_joints, arm_joints;
  
  if(!ReadDHFromFiles(base_joints, arm_joints))
    return;

  if (target_joint_ == davinci::ECM){
    
    API_ECM ecm;

    for (std::size_t i = 0; i < base_joints.size(); ++i){
      ecm.sj_joint_angles[i] = base_joints[i] + base_offsets_[i];
    }

    for (std::size_t i = 0; i < arm_joints.size(); ++i){
      ecm.jnt_pos[i] = arm_joints[i] + arm_offsets_[i];
    }
    
    buildKinematicChainECM1(chain_, ecm, model_.Shaft().transform_);

  }

  else if (target_joint_ == davinci::PSM1 || target_joint_ == davinci::PSM2){

    API_PSM psm;

    for (std::size_t i = 0; i < base_joints.size(); ++i){
      psm.sj_joint_angles[i] = base_joints[i] + base_offsets_[i];
    }

    for (std::size_t i = 0; i < arm_joints.size(); ++i){
      psm.jnt_pos[i] = arm_joints[i] + arm_offsets_[i];
    }

    if (target_joint_ == davinci::PSM1)
      buildKinematicChainPSM1(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);
    else if (target_joint_ == davinci::PSM2)
      buildKinematicChainPSM2(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);

  }

}

bool DHDaVinciPoseGrabber::ReadDHFromFiles(std::vector<double> &psm_base_joints, std::vector<double> &psm_arm_joints){

  try{
    for (int i = 0; i < num_arm_joints_; ++i){
      double x;
      arm_ifs_ >> x;
      psm_arm_joints.push_back(x);
    }

    for (int i = 0; i < num_base_joints_; ++i){
      double x;
      base_ifs_ >> x;
      psm_base_joints.push_back(x);
    }

  }
  catch (std::ifstream::failure){
    do_draw_ = false;
    return false;
  }

  return true;

}

SE3DaVinciPoseGrabber::SE3DaVinciPoseGrabber(const ConfigReader &reader) : BaseDaVinciPoseGrabber(reader) {

  if (reader.get_element("joint") == "PSM1")
    target_joint_ = davinci::DaVinciJoint::PSM1;
  else if (reader.get_element("joint") == "PSM2")
    target_joint_ = davinci::DaVinciJoint::PSM2;
  else if (reader.get_element("joint") == "ECM")
    target_joint_ = davinci::DaVinciJoint::ECM;
  else
    throw std::runtime_error("Error, bad joint");

  ifs_.open(reader.get_element("pose-file"));

  num_wrist_joints_ = 3; //should this load from config file?

}

void SE3DaVinciPoseGrabber::LoadPose(){
  
  ci::Matrix44f next_pose;
  next_pose.setToIdentity();
  std::vector<double> wrist_joints;

  do_draw_ = false; //set to true only if we read a 'good' pose

  try{
    std::string line;
    int row = 0;
    while (1)
    {
      std::getline(ifs_, line);
      if (row == 4) break;
      if (line[0] == '#' || line.length() < 1) continue;
      std::stringstream ss(line);
      for (int col = 0; col < 4; ++col){
        float val;
        ss >> val;
        next_pose.at(row, col) = val;
      }
      row++;
    }
    do_draw_ = true;

    for (int i = 0; i < num_wrist_joints_; ++i){
      double x;
      ifs_ >> x;
      wrist_joints.push_back(x);
    }

  }
  catch (std::ofstream::failure e){
    next_pose.setToIdentity();
    do_draw_ = false;
  }

  if (do_draw_ == false) return;

  model_.Shaft().transform_ = next_pose;

  API_PSM psm;
  for (size_t i = 0; i < wrist_joints.size(); ++i){
    psm.jnt_pos[i] = wrist_joints[i];
  }

  if (target_joint_ == davinci::PSM1)
    buildKinematicChainAtEndPSM1(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);
  else if (target_joint_ == davinci::PSM2)
    buildKinematicChainAtEndPSM2(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);

}