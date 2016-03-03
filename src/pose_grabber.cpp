/**

viz - A robotics visualizer specialized for the da Vinci robotic system.
Copyright (C) 2014 Max Allan

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

**/

#include <cinder/Quaternion.h>
#include <cinder/app/App.h>

#include "davinci.hpp"
#include "../include/pose_grabber.hpp"

using namespace viz;

inline void clean_string(std::string &str, const std::vector<char> &to_remove){
  for (auto &ch : to_remove){
    str.erase(std::remove(str.begin(), str.end(), ch), str.end());
  }
}

size_t BasePoseGrabber::grabber_num_id_ = 0;


std::string BasePoseGrabber::WriteSE3ToString(const ci::Matrix44f &mat){

  std::stringstream ss;

  for (int r = 0; r < 4; ++r){
    ss << "| ";
    for (int c = 0; c < 4; ++c){
      ss << mat.at(r, c) << " ";
    }
    ss << "|\n";
  }

  return ss.str();

}


BasePoseGrabber::BasePoseGrabber(const std::string &output_dir) : do_draw_(false) , save_dir_(output_dir) {

  std::stringstream ss;
  ss << "Pose grabber " << grabber_num_id_;
  param_modifier_ = ci::params::InterfaceGl::create(ci::app::getWindow(), ss.str(), ci::app::toPixels(ci::Vec2i(50, 50)));
  param_modifier_->hide();  

  grabber_num_id_++;

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

PoseGrabber::PoseGrabber(const ConfigReader &reader, const std::string &output_dir) : BasePoseGrabber(output_dir) {

  self_name_ = "pose-grabber";
  checkSelfName(reader.get_element("name"));

  try{
    model_.LoadData(reader.get_element("model-file"));
  }
  catch (std::runtime_error){
    //e.g. no model
  }
  
  
  ifs_.open(reader.get_element("pose-file"));

  if (!ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + reader.get_element("pose-file"));
  }

  ifs_.exceptions(std::ifstream::eofbit);

  save_dir_ = output_dir;

  ofs_file_ = save_dir_ + "/" + reader.get_element("output-pose-file");

}

bool PoseGrabber::LoadPose(const bool update_as_new){

  do_draw_ = false; //set to true only if we read a 'good' pose

  //load the new pose (if requested).
  if (update_as_new){
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
      return false;
    }
  }

  // update the model with the pose
  std::vector<ci::Matrix44f> ret({ cached_model_pose_ });
  model_.SetTransformSet(ret);

  return true;

}

void PoseGrabber::WritePoseToStream()  {

  if (!ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    ofs_.open(ofs_file_);
  }

  if (!ofs_.is_open()) throw(std::runtime_error("Error, cannot open file"));

  ofs_ << model_.Body().transform_ << "\n";

  ofs_ << "\n";

}

void PoseGrabber::WritePoseToStream(const ci::Matrix44f &camera_pose)  {

  if (!ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    ofs_.open(ofs_file_);
  }

  if (!ofs_.is_open()) throw(std::runtime_error("Error, cannot open file"));

  ofs_ << camera_pose.inverted() * model_.Body().transform_ << "\n";

  ofs_ << "\n";

}

BaseDaVinciPoseGrabber::BaseDaVinciPoseGrabber(const ConfigReader &reader, const std::string &output_dir) : BasePoseGrabber(output_dir) {
  
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

DHDaVinciPoseGrabber::DHDaVinciPoseGrabber(const ConfigReader &reader, const std::string &output_dir) : BaseDaVinciPoseGrabber(reader, output_dir) {

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
    num_arm_joints_ = 4;//chain_.mECM1OriginECM1Tip.size(); 
    break;
  }

  arm_offsets_ = std::vector<double>(num_arm_joints_, 0.0);
  base_offsets_ = std::vector<double>(num_base_joints_, 0.0);
  arm_joints_ = std::vector<double>(num_arm_joints_, 0.0);
  base_joints_ = std::vector<double>(num_base_joints_, 0.0);

  try{
    SetupOffsets(reader.get_element("base-offset"), reader.get_element("arm-offset"));
  }
  catch (std::runtime_error &){
    
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

  base_ofs_file_ = output_dir + "/" + reader.get_element("output-base-joint-file");
  arm_ofs_file_ = output_dir + "/" + reader.get_element("output-arm-joint-file");
  try{
    se3_ofs_file_ = output_dir + "/" + reader.get_element("output-se3-file");
  }
  catch (...){
    se3_ofs_file_ = output_dir + "/" + reader.get_element("output-se3"); //stupid old code 
  }
  
}

void DHDaVinciPoseGrabber::SetupOffsets(const std::string &base_offsets, const std::string &arm_offsets){

  std::stringstream ss;
  
  param_modifier_->addText("", "label=`Edit the set up joints`");

  ss << base_offsets;
  for (size_t i = 0; i < base_offsets_.size(); ++i){
    ss >> base_offsets_[i];
    std::stringstream ss;
    ss << "SU Joint " << i;
    param_modifier_->addParam(ss.str(), &(base_offsets_[i]), "min=-10 max=10 step= 0.0001 keyIncr=z keyDecr=Z");
  }

  param_modifier_->addSeparator();
  param_modifier_->addText("", "label=`Edit the arm joints`");

  ss.clear();
  ss << arm_offsets;
  for (size_t i = 0; i < arm_offsets_.size(); ++i){
    ss >> arm_offsets_[i];
    std::stringstream ss;
    ss << "Joint " << i;
    if (i < 3)
      param_modifier_->addParam(ss.str(), &(arm_offsets_[i]), "min=-10 max=10 step= 0.0001 keyIncr=z keyDecr=Z");
    else
      param_modifier_->addParam(ss.str(), &(arm_offsets_[i]), "min=-10 max=10 step= 0.001 keyIncr=z keyDecr=Z");
  }

}

ci::Matrix44f DHDaVinciPoseGrabber::GetPose(){

  if (target_joint_ == davinci::ECM){

    viz::davinci::ECMData ecm;

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

    viz::davinci::PSMData psm;

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

bool DHDaVinciPoseGrabber::LoadPose(const bool update_as_new){

  if (update_as_new){
    if (!ReadDHFromFiles(base_joints_, arm_joints_))
      return false;
  }

  //don't care about the return.
  GetPose();

  // update the list of previous poses for plotting trajectories.
  if (update_as_new){
    reference_frame_tracks_.push_back(model_.Shaft().transform_);
  }

  return true;

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

void DHDaVinciPoseGrabber::WritePoseToStream()  {

  if (!se3_ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    se3_ofs_.open(se3_ofs_file_);
    arm_ofs_.open(arm_ofs_file_);
    base_ofs_.open(base_ofs_file_);
  }

  if (!se3_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));
  if (!arm_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));
  if (!base_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));

  se3_ofs_ << WriteSE3ToString(model_.Shaft().transform_) << "\n";
  for (size_t i = 4; i < arm_joints_.size(); ++i){
    se3_ofs_ << arm_joints_[i] + arm_offsets_[i] << "\n";
  }
  se3_ofs_ << std::endl;

  for (size_t i = 0; i < arm_joints_.size(); ++i){
    arm_ofs_ << arm_joints_[i] + arm_offsets_[i] << " ";
  }
  arm_ofs_ << std::endl;

  for (size_t i = 0; i < base_joints_.size(); ++i){
    base_ofs_ << base_joints_[i] + base_offsets_[i] << " ";
  }
  base_ofs_ << std::endl;

}

void DHDaVinciPoseGrabber::SetOffsetsToNull() {

  for (size_t i = 0; i < base_offsets_.size(); ++i){

    base_offsets_[i] = 0;

  }

  for (size_t i = 0; i < arm_offsets_.size(); ++i){

    arm_offsets_[i] = 0;

  }

}

void DHDaVinciPoseGrabber::WritePoseToStream(const ci::Matrix44f &camera_pose)  {

  if (!se3_ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    se3_ofs_.open(se3_ofs_file_);
    arm_ofs_.open(arm_ofs_file_);
    base_ofs_.open(base_ofs_file_);
  }

  if (!se3_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));
  if (!arm_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));
  if (!base_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));

  se3_ofs_ << WriteSE3ToString(camera_pose.inverted() * model_.Shaft().transform_) << "\n";
  for (size_t i = 4; i < arm_joints_.size(); ++i){
    se3_ofs_ << arm_joints_[i] + arm_offsets_[i] << "\n";
  }
  se3_ofs_ << std::endl;

  for (size_t i = 0; i < arm_joints_.size(); ++i){
    arm_ofs_ << arm_joints_[i] + arm_offsets_[i] << " ";
  }
  arm_ofs_ << std::endl;

  for (size_t i = 0; i < base_joints_.size(); ++i){
    base_ofs_ << base_joints_[i] + base_offsets_[i] << " ";
  }
  base_ofs_ << std::endl;

}

void SE3DaVinciPoseGrabber::SetupOffsets(const std::string &base_offsets, const std::string &arm_offsets){

  //if (base_offsets.size() != 6 || arm_offsets.size() != 3) {
  //  ci::app::console() << "Error, base offsets size should be 6 and arm offsets size should be 3\n" << std::endl;
  //  throw std::runtime_error("");
  //}

  std::stringstream ss;
  std::stringstream ss2;

  param_modifier_->addText("", "label=`Edit the 6 DOF pose joints`");

  ss << base_offsets;
  ss >> x_rotation_offset_;
  param_modifier_->addParam("X rotation offset", &x_rotation_offset_, "min=-10 max=10 step= 0.01 keyIncr=r keyDecr=R");
  

  ss << base_offsets;
  ss >> y_rotation_offset_;
  param_modifier_->addParam("Y rotation offset", &y_rotation_offset_, "min=-10 max=10 step= 0.01 keyIncr=p keyDecr=P");

  ss << base_offsets;
  ss >> z_rotation_offset_;
  param_modifier_->addParam("Z rotation offset", &z_rotation_offset_, "min=-10 max=10 step= 0.01 keyIncr=y keyDecr=Y");

  ss << base_offsets;
  ss >> x_translation_offset_;
  param_modifier_->addParam("X translation offset", &x_translation_offset_, "min=-10 max=10 step= 0.001 keyIncr=x keyDecr=X");

  ss << base_offsets;
  ss >> y_translation_offset_;
  param_modifier_->addParam("Y translation offset", &y_translation_offset_, "min=-10 max=10 step= 0.001 keyIncr=y keyDecr=Y");

  ss << base_offsets;
  ss >> z_translation_offset_;
  param_modifier_->addParam("Z translation offset", &z_translation_offset_, "min=-10 max=10 step= 0.001 keyIncr=z keyDecr=Z");

  param_modifier_->addSeparator();
  param_modifier_->addText("", "label=`Edit the wrist joints`");

  ss.clear();
  ss << arm_offsets;
  for (size_t i = 0; i < wrist_offsets_.size(); ++i){
    ss >> wrist_offsets_[i];
    std::stringstream ss2;
    ss2 << "Joint " << i;
    if (i < 3)
      param_modifier_->addParam(ss2.str(), &(wrist_offsets_[i]), "min=-10 max=10 step= 0.0001 keyIncr=z keyDecr=Z");
    else
      param_modifier_->addParam(ss2.str(), &(wrist_offsets_[i]), "min=-10 max=10 step= 0.001 keyIncr=z keyDecr=Z");
  }

}

void SE3DaVinciPoseGrabber::SetOffsetsToNull() {

  x_rotation_offset_ = 0.0f;
  y_rotation_offset_ = 0.0f;
  z_rotation_offset_ = 0.0f;
  x_translation_offset_ = 0.0f;
  y_translation_offset_ = 0.0f;
  z_translation_offset_ = 0.0f;

  for (int i = 0; i < num_wrist_joints_; ++i){
    wrist_offsets_[i] = 0.0f;
  }

}

SE3DaVinciPoseGrabber::SE3DaVinciPoseGrabber(const ConfigReader &reader, const std::string &output_dir, bool check_type) : BaseDaVinciPoseGrabber(reader, output_dir) {

  if (check_type){
    self_name_ = "se3-davinci-grabber";
    checkSelfName(reader.get_element("name"));
  }

  if (reader.get_element("joint") == "PSM1")
    target_joint_ = davinci::DaVinciJoint::PSM1;
  else if (reader.get_element("joint") == "PSM2")
    target_joint_ = davinci::DaVinciJoint::PSM2;
  else if (reader.get_element("joint") == "ECM")
    target_joint_ = davinci::DaVinciJoint::ECM;
  else
    throw std::runtime_error("Error, bad joint");

  ifs_.open(reader.get_element("pose-file"));
  if (!ifs_.is_open()) throw std::runtime_error("Could not open file!\n");
  ofs_file_ = output_dir + "/" + reader.get_element("output-pose-file");

  num_wrist_joints_ = 3; //should this load from config file?

  wrist_dh_params_ = std::vector<double>(num_wrist_joints_, 0.0);
  wrist_offsets_ = std::vector<float>(num_wrist_joints_, 0.0);

  try{
    SetupOffsets(reader.get_element("base-offset"), reader.get_element("arm-offset"));
  }
  catch (std::runtime_error &){

  }

}

inline ci::Vec3f EulersFromQuaternion(const ci::Quatf &q){

  float roll = ci::math<float>::atan2(2.0f * (q.v.y * q.v.z + q.w * q.v.x), 1 - 2*(q.v.x * q.v.x + q.v.y*q.v.y));
  float pitch = ci::math<float>::asin(2.0f * (q.w * q.v.y - q.v.x * q.v.z));
  float yaw = ci::math<float>::atan2(2.0f * (q.v.x * q.v.y + q.w * q.v.z), 1 - 2*(q.v.y * q.v.y + q.v.z * q.v.z));
  return ci::Vec3f(roll, pitch, yaw);
}

inline ci::Matrix44f MatrixFromClassicEulers(float xRotation, float yRotation, float zRotation){


}

inline ci::Matrix44f MatrixFromIntrinsicEulers(float xRotation, float yRotation, float zRotation){

  float cosx = ci::math<float>::cos(xRotation);
  float cosy = ci::math<float>::cos(yRotation);
  float cosz = ci::math<float>::cos(zRotation);
  float sinx = ci::math<float>::sin(xRotation);
  float siny = ci::math<float>::sin(yRotation);
  float sinz = ci::math<float>::sin(zRotation);

  ci::Matrix33f xRotationMatrix; xRotationMatrix.setToIdentity();
  ci::Matrix33f yRotationMatrix; yRotationMatrix.setToIdentity();
  ci::Matrix33f zRotationMatrix; zRotationMatrix.setToIdentity();

  xRotationMatrix.at(1, 1) = xRotationMatrix.at(2, 2) = cosx;
  xRotationMatrix.at(1, 2) = -sinx;
  xRotationMatrix.at(2, 1) = sinx;

  yRotationMatrix.at(0, 0) = yRotationMatrix.at(2, 2) = cosy;
  yRotationMatrix.at(0, 2) = siny;
  yRotationMatrix.at(2, 0) = -siny;

  zRotationMatrix.at(0, 0) = zRotationMatrix.at(1, 1) = cosz;
  zRotationMatrix.at(0, 1) = -sinz;
  zRotationMatrix.at(1, 0) = sinz;

  //ci::Matrix33f r = zRotationMatrix * yRotationMatrix * xRotationMatrix;

  ci::Matrix33f r = xRotationMatrix * yRotationMatrix * zRotationMatrix;

  ci::Matrix44f rr = r;
  rr.at(3, 3) = 1.0f;
  return rr;

  ci::Matrix44f m;
  
  // Tait-Bryan angles X_1, Y_2, Z_3
  //m.at(0, 0) = cosy * cosz;
  //m.at(0, 1) = -cosy * sinz;
  //m.at(0, 2) = siny;
  //m.at(1, 0) = cosx * sinz + cosz * sinx * siny;
  //m.at(1, 1) = cosx * cosz - sinx * siny * sinz;
  //m.at(1, 2) = -cosy * sinx;
  //m.at(2, 0) = sinx * sinz - cosx * cosz * siny;
  //m.at(2, 1) = cosz * sinx + cosx * siny * sinz;
  //m.at(2, 2) = cosx * cosy;

  m.at(0, 0) = cosy * cosz;
  m.at(0, 1) = sinx * siny * cosz - cosx * sinz;
  m.at(0, 2) = sinx * sinz + cosx * siny * cosz;
  m.at(1, 0) = cosy * sinz;
  m.at(1, 1) = cosx * cosz + sinx * siny * sinz;
  m.at(1, 2) = cosx * siny * sinz - sinx * cosz;
  m.at(2, 0) = -siny;
  m.at(2, 1) = sinx * cosy;
  m.at(2, 2) = cosx * cosy;

  return m;

}

inline ci::Quatf QuaternionFromEulers(float xRotation, float yRotation, float zRotation){

  zRotation *= 0.5f;
  yRotation *= 0.5f;
  xRotation *= 0.5f;

  // get sines and cosines of half angles
  float Cx = ci::math<float>::cos(xRotation);
  float Sx = ci::math<float>::sin(xRotation);

  float Cy = ci::math<float>::cos(yRotation);
  float Sy = ci::math<float>::sin(yRotation);
                            
  float Cz = ci::math<float>::cos(zRotation);
  float Sz = ci::math<float>::sin(zRotation);

  ci::Quatf r;
  // multiply it out
  r.w = Cx*Cy*Cz + Sx*Sy*Sz;
  r.v.x = Sx*Cy*Cz - Cx*Sy*Sz;
  r.v.y = Cx*Sy*Cz + Sx*Cy*Sz;
  r.v.z = Cx*Cy*Sz - Sx*Sy*Cz;

  return r.normalized();
}


bool SE3DaVinciPoseGrabber::LoadPose(const bool update_as_new){
  
  do_draw_ = false; //set to true only if we read a 'good' pose

  assert(num_wrist_joints_ == wrist_dh_params_.size());

  static ci::Matrix44f current_user_supplied_offset;

  if (update_as_new){

    try{
      std::string line;
      int row = 0;

      ci::Vec3f articulation;
      //remember - also set psmatend rotation angle for tip to +- val rather than +- 0.5*val. aslo skipping frist 59 frames.


      for (int i = 0; i < 3; ++i){
        ifs_ >> translation_[i];
      }

      for (int i = 0; i < 4; ++i){
        ifs_ >> rotation_[i];
      }

      for (int i = 0; i < 3; ++i){
        ifs_ >> articulation[i];
      }
      for (int i = 0; i < 3; ++i){
        wrist_dh_params_[i] = articulation[i];
      }

      shaft_pose_ = rotation_ * current_user_supplied_offset;

    }
    catch (std::ifstream::failure e){
      shaft_pose_.setToIdentity();
      do_draw_ = false;
      return false;
    }
  }
  static float x_rotation_offsetQ = x_rotation_offset_;
  static float y_rotation_offsetQ = y_rotation_offset_;
  static float z_rotation_offsetQ = z_rotation_offset_;

  auto offset = MatrixFromIntrinsicEulers(x_rotation_offset_ - x_rotation_offsetQ, y_rotation_offset_ - y_rotation_offsetQ, z_rotation_offset_ - z_rotation_offsetQ);

  x_rotation_offsetQ = x_rotation_offset_;
  y_rotation_offsetQ = y_rotation_offset_;
  z_rotation_offsetQ = z_rotation_offset_;

  current_user_supplied_offset = current_user_supplied_offset * offset;

  shaft_pose_ = shaft_pose_ * offset;

  ci::app::console() << "Shaft pose = " << shaft_pose_ << std::endl;

  shaft_pose_.setTranslate(translation_ + ci::Vec3f(x_translation_offset_, y_translation_offset_, z_translation_offset_));
  do_draw_ = true;

  // update the list of previous poses for plotting trajectories.
  reference_frame_tracks_.push_back(shaft_pose_);

  model_.Shaft().transform_ = shaft_pose_;

  viz::davinci::PSMData psm;
  for (size_t i = 0; i < num_wrist_joints_; ++i){
    psm.jnt_pos[i] = wrist_dh_params_[i] + wrist_offsets_[i];
  }

  if (target_joint_ == davinci::PSM1)
    buildKinematicChainAtEndPSM1(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);
  else if (target_joint_ == davinci::PSM2)
    buildKinematicChainAtEndPSM2(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);

  return true;

}

void SE3DaVinciPoseGrabber::WritePoseToStream() {

  if (!ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    ofs_.open(ofs_file_);
  }

  if (!ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));

  ofs_ << WriteSE3ToString(model_.Shaft().transform_) << "\n";
  
  for (size_t i = 0; i < wrist_dh_params_.size(); ++i){
    ofs_ << wrist_dh_params_[i] << "\n";
  }
  ofs_ << "\n";
}

void SE3DaVinciPoseGrabber::WritePoseToStream(const ci::Matrix44f &camera_pose)  {

  if (!ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    ofs_.open(ofs_file_);
  }

  if (!ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));

  ofs_ << WriteSE3ToString(camera_pose.inverted() * model_.Shaft().transform_) << "\n";
  for (size_t i = 0; i < wrist_dh_params_.size(); ++i){
    ofs_ << wrist_dh_params_[i] << "\n";
  }
  ofs_ << "\n";
  
}

void DHDaVinciPoseGrabber::GetModelPose(ci::Matrix44f &head, ci::Matrix44f &clasper_left, ci::Matrix44f &clasper_right){
	
	if (target_joint_ == davinci::PSM1 || target_joint_ == davinci::PSM2){

		viz::davinci::PSMData psm;

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

		head = model_.Head().transform_;
		clasper_left = model_.Clasper1().transform_;
		clasper_right = model_.Clasper2().transform_;

	}
	else{

		throw std::runtime_error("Error, bad joint type");

	}

}

void SE3DaVinciPoseGrabber::GetModelPose(ci::Matrix44f &head, ci::Matrix44f &clasper_left, ci::Matrix44f &clasper_right){

  if (target_joint_ == davinci::PSM1 || target_joint_ == davinci::PSM2){
  
    model_.Shaft().transform_ = shaft_pose_;

    viz::davinci::PSMData psm;
    for (size_t i = 0; i < num_wrist_joints_; ++i){
      psm.jnt_pos[i] = wrist_dh_params_[i];
    }
    
    if (target_joint_ == davinci::PSM1)
      buildKinematicChainPSM1(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);
    else if (target_joint_ == davinci::PSM2)
      buildKinematicChainPSM2(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);

    head = model_.Head().transform_;
    clasper_left = model_.Clasper1().transform_;
    clasper_right = model_.Clasper2().transform_;

  }
  else{

    throw std::runtime_error("Error, bad joint type");

  }

}

void SE3DaVinciPoseGrabber::DrawBody(){

  model_.DrawBody();

}

void SE3DaVinciPoseGrabber::DrawHead(){

  model_.DrawHead();
  model_.DrawLeftClasper();
  model_.DrawRightClasper();

}

void DHDaVinciPoseGrabber::DrawBody(){

	model_.DrawBody();

}

void DHDaVinciPoseGrabber::DrawHead(){

	model_.DrawHead();
  model_.DrawLeftClasper();
  model_.DrawRightClasper();

}

bool QuaternionPoseGrabber::LoadPose(const bool update_as_new){

  do_draw_ = false; //set to true only if we read a 'good' pose

  //load the new pose (if requested).
  if (update_as_new){
    try{
      std::string line;
      int row = 0;
      while (ifs_.good()){
        std::getline(ifs_, line);
        if (line[0] == '#' || line.length() < 1) continue;
        break;
      }
      std::stringstream ss(line);
      
      ci::Vec3f translation;
      for (size_t col = 0; col < 3; ++col){
        float val;
        ss >> val;
        translation[col] = val;
      }

      shaft_pose_.setTranslate(translation);

      ci::Vec4f quats;
      for (size_t col = 0; col < 4; ++col){
        float val;
        ss >> val;
        quats[col] = val;
      }

      shaft_pose_ = ci::Quatf(quats[0], quats[1], quats[2], quats[3]);

      

      //update the reference list of old tracks for drawing trajectories
      reference_frame_tracks_.push_back(shaft_pose_);
      do_draw_ = true;

    }
    catch (std::ofstream::failure e){
      shaft_pose_.setToIdentity();
      do_draw_ = false;
      return false;
    }
  }

  // update the model with the pose
  model_.Shaft().transform_ = shaft_pose_;

  viz::davinci::PSMData psm;
  for (size_t i = 0; i < num_wrist_joints_; ++i){
    psm.jnt_pos[i] = wrist_dh_params_[i];
  }

  if (target_joint_ == davinci::PSM1)
    buildKinematicChainAtEndPSM1(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);
  else if (target_joint_ == davinci::PSM2)
    buildKinematicChainAtEndPSM2(chain_, psm, model_.Shaft().transform_, model_.Head().transform_, model_.Clasper1().transform_, model_.Clasper2().transform_);

  return true;


}

QuaternionPoseGrabber::QuaternionPoseGrabber(const ConfigReader &reader, const std::string &output_dir) : SE3DaVinciPoseGrabber(reader, output_dir, false) {

  self_name_ = "quaternion-pose-grabber";
  checkSelfName(reader.get_element("name"));

}