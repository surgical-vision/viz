#include "davinci.hpp"

using namespace viz::davinci;

float PI = 3.14159265358979323846f;
float PI_2 = 1.57079632679489661923f;
float PI_4 = 0.785398163397448309616f;

/*
void DaVinciKinematicChain::UpdateChain(std::vector<double> &suj_joints, std::vector<double> &j_joints, DaVinciJoint joint_type){

  if (joint_type == PSM1){

    for (std::size_t i = 0; i < suj_joints.size(); ++i){
      if (mSUJ1OriginSUJ1Tip[i].mJointType == JointTypeEnum::ROTARY)
        mSUJ1OriginSUJ1Tip[i].mTheta = suj_joints[i];
      if (mSUJ1OriginSUJ1Tip[i].mJointType == JointTypeEnum::PRISMATIC)
        mSUJ1OriginSUJ1Tip[i].mD = suj_joints[i];
      if (mSUJ1OriginSUJ1Tip[i].mJointType == JointTypeEnum::FIXED)
        continue;
    }

    for (std::size_t i = 0; i < j_joints.size(); ++i){
      if (mPSM1OriginPSM1Tip[i].mJointType == JointTypeEnum::ROTARY)
        mPSM1OriginPSM1Tip[i].mTheta = j_joints[i];
      if (mPSM1OriginPSM1Tip[i].mJointType == JointTypeEnum::PRISMATIC)
        mPSM1OriginPSM1Tip[i].mD = j_joints[i];
      if (mPSM1OriginPSM1Tip[i].mJointType == JointTypeEnum::FIXED)
        continue;
    }

  }

  else if (joint_type == PSM2){

    for (std::size_t i = 0; i < suj_joints.size(); ++i){
      if (mSUJ2OriginSUJ2Tip[i].mJointType == JointTypeEnum::ROTARY)
        mSUJ2OriginSUJ2Tip[i].mTheta = suj_joints[i];
      if (mSUJ2OriginSUJ2Tip[i].mJointType == JointTypeEnum::PRISMATIC)
        mSUJ2OriginSUJ2Tip[i].mD = suj_joints[i];
      if (mSUJ2OriginSUJ2Tip[i].mJointType == JointTypeEnum::FIXED)
        continue;
    }

    for (std::size_t i = 0; i < j_joints.size(); ++i){
      if (mPSM2OriginPSM2Tip[i].mJointType == JointTypeEnum::ROTARY)
        mPSM2OriginPSM2Tip[i].mTheta = j_joints[i];
      if (mPSM2OriginPSM2Tip[i].mJointType == JointTypeEnum::PRISMATIC)
        mPSM2OriginPSM2Tip[i].mD = j_joints[i];
      if (mPSM2OriginPSM2Tip[i].mJointType == JointTypeEnum::FIXED)
        continue;
    }

  }

  else if (joint_type == ECM){

    for (std::size_t i = 0; i < suj_joints.size(); ++i){
      if (mSUJ3OriginSUJ3Tip[i].mJointType == JointTypeEnum::ROTARY)
        mSUJ3OriginSUJ3Tip[i].mTheta = suj_joints[i];
      if (mSUJ3OriginSUJ3Tip[i].mJointType == JointTypeEnum::PRISMATIC)
        mSUJ3OriginSUJ3Tip[i].mD = suj_joints[i];
      if (mSUJ3OriginSUJ3Tip[i].mJointType == JointTypeEnum::FIXED)
        continue;
    }

    for (std::size_t i = 0; i < j_joints.size(); ++i){
      if (mECM1OriginECM1Tip[i].mJointType == JointTypeEnum::ROTARY)
        mECM1OriginECM1Tip[i].mTheta = j_joints[i];
      if (mECM1OriginECM1Tip[i].mJointType == JointTypeEnum::PRISMATIC)
        mECM1OriginECM1Tip[i].mD = j_joints[i];
      if (mECM1OriginECM1Tip[i].mJointType == JointTypeEnum::FIXED)
        continue;
    }

  }

  else{
    throw std::runtime_error("Error!");
  }

}
*/

DaVinciKinematicChain::DaVinciKinematicChain(void){
  
  // Standard angles
  float PI = 3.14159265358979323846f;
  float PI_2 = 1.57079632679489661923f;
  float PI_4 = 0.785398163397448309616f;

  // General transformation from world origin to SUJ1 origin (assumes alpha cart)
  mWorldOriginSUJ1Origin.push_back(GeneralFrame(-0.1016f, -0.1016f, 0.43f, -1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from SUJ1 origin to SUJ1 tip (assumes alpha cart)
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::PRISMATIC, 0.08979f, 0.0f, 0.0f, 0.0f));
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, 0.0f, 0.4166f, 0.0f));
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::ROTARY, 0.4318f, 0.0f, 0.14288f, 0.0f));
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.4318f, 0.0f, -0.1302f, PI_2));
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::ROTARY, 0.0f, PI_2, 0.4089f, 0.0f));
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::ROTARY, 0.0f, -PI_2, -0.1029f, -PI_2));

  // General transformationfrom SUJ1 tip to PSM1 origin
  mSUJ1TipPSM1Origin.push_back(GeneralFrame(0.478f, 0.0f, 0.1524f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from PSM1 origin to PSM1 tip (assumes needle driver instrument)
  float PSM1_lenRCC = 0.4318f;
  float PSM1_ToolLen = 0.4159f;
  float PSM1_PitchToYaw = 0.009f;
  float PSM1_YawToCtrlPnt = 0.0f;
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::ROTARY, 0.0f, PI_2, 0.0f, PI_2));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, -PI_2, 0.0f, -PI_2));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::PRISMATIC, 0.0f, PI_2, -PSM1_lenRCC, 0.0f));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.0f, 0.0f, PSM1_ToolLen, 0.0f));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::ROTARY, 0.0f, -PI_2, 0.0f, -PI_2));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::ROTARY, PSM1_PitchToYaw, -PI_2, 0.0f, -PI_2));
  //mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(7, JointTypeEnum::FIXED, 0.0f, -PI_2, PSM1_YawToCtrlPnt, 0.0f));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(7, JointTypeEnum::ROTARY, 0.0f, -PI_2, PSM1_YawToCtrlPnt, 0.0f));

  // General transformation from world origin to SUJ2 origin (assumes alpha cart)
  mWorldOriginSUJ2Origin.push_back(GeneralFrame(0.1016f, -0.1016f, 0.43f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from SUJ2 origin to SUJ2 tip (assumes alpha cart)
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::PRISMATIC, 0.08979f, 0.0f, 0.0f, 0.0f));
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, 0.0f, 0.4166f, 0.0f));
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::ROTARY, 0.4318f, 0.0f, 0.14288f, 0.0f));
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.4318f, 0.0f, -0.1302f, PI_2));
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::ROTARY, 0.0f, PI_2, 0.4089f, 0.0f));
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::ROTARY, 0.0f, -PI_2, -0.1029f, -PI_2));

  // General transformationfrom SUJ2 tip to PSM2 origin
  mSUJ2TipPSM2Origin.push_back(GeneralFrame(0.478f, 0.0f, 0.1524f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from PSM2 origin to PSM2 tip (assumes needle driver instrument)
  float PSM2_lenRCC = 0.4318f;
  float PSM2_ToolLen = 0.4159f;
  float PSM2_PitchToYaw = 0.009f;
  float PSM2_YawToCtrlPnt = 0.0f;
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::ROTARY, 0.0f, PI_2, 0.0f, PI_2));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, -PI_2, 0.0f, -PI_2));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::PRISMATIC, 0.0f, PI_2, -PSM2_lenRCC, 0.0f));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.0f, 0.0f, PSM2_ToolLen, 0.0f));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::ROTARY, 0.0f, -PI_2, 0.0f, -PI_2));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::ROTARY, PSM2_PitchToYaw, -PI_2, 0.0f, -PI_2));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(7, JointTypeEnum::FIXED, 0.0f, -PI_2, PSM2_YawToCtrlPnt, 0.0f));

  // Denavit-Hartenberg parameters from world origin to SUJ3 origin (assumes alpha cart)
  mWorldOriginSUJ3Origin.push_back(GeneralFrame(0.0f, 0.0f, 0.43f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from SUJ3 origin to SUJ3 tip (assumes alpha cart)
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::PRISMATIC, 0.08979f, 0.0f, 0.0f, 0.0f));
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, 0.0f, 0.4166f, 0.0f));
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::ROTARY, 0.4318f, 0.0f, 0.14288f, 0.0f));
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.4318f, 0.0f, -0.34588f, PI_2));
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::FIXED, 0.0f, -PI_4, 0.0f, PI_2));
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::FIXED, -0.06641f, 0.0f, 0.0f, 0.0f));

  // Denavit-Hartenberg parameters from SUJ3 tip to ECM1 origin
  mSUJ3TipECM1Origin.push_back(GeneralFrame(0.6126f, 0.0f, 0.1016f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from ECM1 origin to ECM1 tip (assumes Olympus 0 degree stereo scope)
  float ECM1_lenRCC = 0.3822f;
  float ECM1_ScopeLen = 0.3828f;
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::ROTARY, 0.0f, PI_2, 0.0f, PI_2));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, -PI_2, 0.0f, -PI_2));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::PRISMATIC, 0.0f, PI_2, -ECM1_lenRCC, 0.0f));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.0f, 0.0f, ECM1_ScopeLen, 0.0f));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::FIXED, 0.0f, -PI_2, 0.0f, -PI_2));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::FIXED, 0.0f, -PI_2, 0.0f, -PI_2));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(7, JointTypeEnum::FIXED, 0.0f, -PI_2, 0.0f, 0.0f));
  
}
