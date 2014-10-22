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

PoseGrabber::PoseGrabber(const ConfigReader &reader) {

  self_name_ = "pose-grabber";
  checkSelfName(reader.get_element("name"));

  model_.LoadData(reader.get_element("model-file"));
  ifs_.open(reader.get_element("pose-file"));

  if (!ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + reader.get_element("pose-file"));
  }

  ifs_.exceptions(std::ifstream::eofbit);

}

void PoseGrabber::LoadPose(const bool no_reload){

  do_draw_ = false; //set to true only if we read a 'good' pose

  //load the new pose (if requested).
  if (no_reload){
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
          cached_model_pose_.at(row, col) = val;
        }
        row++;
      }

      //update the reference list of old tracks for drawing trajectories
      reference_frame_tracks_.push_back(cached_model_pose_);
      do_draw_ = true;

    }
    catch (std::ofstream::failure e){
      cached_model_pose_.setToIdentity();
      do_draw_ = false;
    }
  }

  // update the model with the pose
  std::vector<ci::Matrix44f> ret({ cached_model_pose_ });
  model_.SetTransformSet(ret);

}

std::string PoseGrabber::writePoseToString() const {

  throw std::runtime_error("");

  return "";

}

std::string PoseGrabber::writePoseToString(const ci::Matrix44f &camera_pose) const {

  throw std::runtime_error("");

  return "";

}

BaseDaVinciPoseGrabber::BaseDaVinciPoseGrabber(const ConfigReader &reader){
  
  try{
    model_.LoadData(reader.get_element("model-file"));
  }
  catch (std::runtime_error){
    //no model (e.g. tracking camera)
  }

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

  self_name_ = "dh-davinci-grabber";
  checkSelfName(reader.get_element("name"));

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

  arm_offsets_ = std::vector<double>(num_arm_joints_, 0.0);
  base_offsets_ = std::vector<double>(num_base_joints_, 0.0);
  arm_joints_ = std::vector<double>(num_arm_joints_, 0.0);
  base_joints_ = std::vector<double>(num_base_joints_, 0.0);

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

}

ci::Matrix44f DHDaVinciPoseGrabber::GetPose(){

  if (target_joint_ == davinci::ECM){

    API_ECM ecm;

    for (std::size_t i = 0; i < base_joints_.size(); ++i){
      ecm.sj_joint_angles[i] = base_joints_[i] + base_offsets_[i];
    }

    for (std::size_t i = 0; i < arm_joints_.size(); ++i){
      ecm.jnt_pos[i] = arm_joints_[i] + arm_offsets_[i];
    }

    buildKinematicChainECM1(chain_, ecm, model_.Shaft().transform_);

    return model_.Shaft().transform_;

  }

  else if (target_joint_ == davinci::PSM1 || target_joint_ == davinci::PSM2){

    API_PSM psm;

    for (std::size_t i = 0; i < base_joints_.size(); ++i){
      psm.sj_joint_angles[i] = base_joints_[i] + base_offsets_[i];
    }

    for (std::size_t i = 0; i < arm_joints_.size(); ++i){
      psm.jnt_pos[i] = arm_joints_[i] + arm_offsets_[i];
    }

    if (target_joint_ == davinci::PSM1)
      buildKinematicChainPSM1(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);
    else if (target_joint_ == davinci::PSM2)
      buildKinematicChainPSM2(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);

    return model_.Shaft().transform_;

  }
  else{

    throw std::runtime_error("Error, bad joint type");

  }
}

void DHDaVinciPoseGrabber::LoadPose(const bool no_reload){

  if (!no_reload){
    if (!ReadDHFromFiles(base_joints_, arm_joints_))
      return;
  }

  //don't care about the return.
  GetPose();

  // update the list of previous poses for plotting trajectories.
  if (!no_reload){
    reference_frame_tracks_.push_back(model_.Shaft().transform_);
  }

}

bool DHDaVinciPoseGrabber::ReadDHFromFiles(std::vector<double> &psm_base_joints, std::vector<double> &psm_arm_joints){

  assert(num_arm_joints_ == psm_arm_joints.size());
  assert(num_base_joints_ == psm_base_joints.size());

  try{
    for (int i = 0; i < num_arm_joints_; ++i){
      double x;
      arm_ifs_ >> x;
      psm_arm_joints[i] = x;
    }

    for (int i = 0; i < num_base_joints_; ++i){
      double x;
      base_ifs_ >> x;
      psm_base_joints[i] = x;
    }

  }
  catch (std::ifstream::failure){
    do_draw_ = false;
    return false;
  }

  return true;

}

std::string DHDaVinciPoseGrabber::writePoseToString() const {

  throw std::runtime_error("");

  return "";

}

std::string DHDaVinciPoseGrabber::writePoseToString(const ci::Matrix44f &camera_pose) const {

  throw std::runtime_error("");

  return "";

}

SE3DaVinciPoseGrabber::SE3DaVinciPoseGrabber(const ConfigReader &reader) : BaseDaVinciPoseGrabber(reader) {

  self_name_ = "se3-davinci-grabber";
  checkSelfName(reader.get_element("name"));

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

  wrist_dh_params_ = std::vector<double>(num_wrist_joints_, 0.0);

}

void SE3DaVinciPoseGrabber::LoadPose(const bool no_reload){
  
  do_draw_ = false; //set to true only if we read a 'good' pose

  assert(num_wrist_joints_ == wrist_dh_params_.size());

  if (!no_reload){

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
          shaft_pose_.at(row, col) = val;
        }
        row++;
      }
      do_draw_ = true;

      for (int i = 0; i < num_wrist_joints_; ++i){
        double x;
        ifs_ >> x;
        wrist_dh_params_[i] = x;
      }

      // update the list of previous poses for plotting trajectories.
      reference_frame_tracks_.push_back(shaft_pose_);

    }
    catch (std::ofstream::failure e){
      shaft_pose_.setToIdentity();
      do_draw_ = false;
    }

  }
  if (do_draw_ == false) return;

  model_.Shaft().transform_ = shaft_pose_;

  API_PSM psm;
  for (size_t i = 0; i < num_wrist_joints_; ++i){
    psm.jnt_pos[i] = wrist_dh_params_[i];
  }

  if (target_joint_ == davinci::PSM1)
    buildKinematicChainAtEndPSM1(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);
  else if (target_joint_ == davinci::PSM2)
    buildKinematicChainAtEndPSM2(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);

}

std::string SE3DaVinciPoseGrabber::writePoseToString() const {

  throw std::runtime_error("");

  return "";

}

std::string SE3DaVinciPoseGrabber::writePoseToString(const ci::Matrix44f &camera_pose) const {

  throw std::runtime_error("");

  return "";

}