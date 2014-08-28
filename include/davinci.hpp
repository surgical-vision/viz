#pragma once 
#include <vector>

namespace viz {

  namespace davinci {

    enum DaVinciJoint { PSM1, PSM2, PSM3, ECM };

    struct JointTypeEnum {
      enum Enum {
        FIXED = 0,
        ROTARY = 1,
        PRISMATIC = 2,
      };
    };


    struct GeneralFrame{
      GeneralFrame() : mX(0.0f), mY(0.0f), mZ(0.0f), mR00(1.0f), mR01(0.0f), mR02(0.0f), mR10(0.0f), mR11(1.0f), mR12(0.0f), mR20(0.0f), mR21(0.0f), mR22(1.0f) {}
      GeneralFrame(float x, float y, float z, float r00, float r01, float r02, float r10, float r11, float r12, float r20, float r21, float r22) :
        mX(x), mY(y), mZ(z), mR00(r00), mR01(r01), mR02(r02), mR10(r10), mR11(r11), mR12(r12), mR20(r20), mR21(r21), mR22(r22) {}
      float mX, mY, mZ;
      float mR00, mR01, mR02;
      float mR10, mR11, mR12;
      float mR20, mR21, mR22;
    };

    struct DenavitHartenbergFrame{
      DenavitHartenbergFrame() : mFrame(0), mJointType(JointTypeEnum::FIXED), mA(0.0f), mAlpha(0.0f), mD(0.0f), mTheta(0.0f){}
      explicit DenavitHartenbergFrame(unsigned int frame, JointTypeEnum::Enum joint_type, float a, float alpha, float d, float theta) :
        mFrame(frame), mJointType(joint_type), mA(a), mAlpha(alpha), mD(d), mTheta(theta){ }
      unsigned int mFrame;
      JointTypeEnum::Enum mJointType;
      float mA;
      float mAlpha;
      float mD;
      float mTheta;
    };





    struct DaVinciKinematicChain {
      // Construction
      DaVinciKinematicChain();

      //void UpdateChain(std::vector<double> &suj_joints, std::vector<double> &j_joints, DaVinciJoint joint_type);

      // PSM1
      std::vector<GeneralFrame> mWorldOriginSUJ1Origin;
      std::vector<DenavitHartenbergFrame> mSUJ1OriginSUJ1Tip;
      std::vector<GeneralFrame> mSUJ1TipPSM1Origin;
      std::vector<DenavitHartenbergFrame> mPSM1OriginPSM1Tip;

      // PSM2
      std::vector<GeneralFrame> mWorldOriginSUJ2Origin;
      std::vector<DenavitHartenbergFrame> mSUJ2OriginSUJ2Tip;
      std::vector<GeneralFrame> mSUJ2TipPSM2Origin;
      std::vector<DenavitHartenbergFrame> mPSM2OriginPSM2Tip;

      // ECM1
      std::vector<GeneralFrame> mWorldOriginSUJ3Origin;
      std::vector<DenavitHartenbergFrame> mSUJ3OriginSUJ3Tip;
      std::vector<GeneralFrame> mSUJ3TipECM1Origin;
      std::vector<DenavitHartenbergFrame> mECM1OriginECM1Tip;
    };


  }


}