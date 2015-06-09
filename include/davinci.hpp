#pragma once 

#include <vector>

#include "camera.hpp"

namespace viz {

  namespace davinci {

    /**
    * @struct PSMData
    * Basic wrapper for the data that comes from a Da Vinci PSM
    */
    struct PSMData {

      float jnt_pos[7]; /**< The arm joint positions. */
      float sj_joint_angles[6]; /**< The set up joint positions. */

    };

    /**
    * @struct ECMData
    * Basic wrapper for the data that comes from a Da Vinci ECM
    */
    struct ECMData {

      float jnt_pos[4]; /**< The arm joint positions. */
      float sj_joint_angles[6];  /**< The set up joint positions. */

    };

    /**
    * @enum DaVinciJoint
    * The arms on a classic da Vinci.
    */
    enum DaVinciJoint { PSM1, PSM2, PSM3, ECM };

    /**
    * @enum JointTypeEnum
    * The types of joint on a da Vinci arm.
    */
    struct JointTypeEnum {
      enum Enum {
        FIXED = 0,
        ROTARY = 1,
        PRISMATIC = 2,
      };
    };

    /**
    * @struct GeneralFrame
    * The general 3D transformation type for a rigid body.
    */
    struct GeneralFrame{

      /**
      * Default constructor. Sets the transform to null translation and identity rotation.
      */
      GeneralFrame() : mX(0.0f), mY(0.0f), mZ(0.0f), mR00(1.0f), mR01(0.0f), mR02(0.0f), mR10(0.0f), mR11(1.0f), mR12(0.0f), mR20(0.0f), mR21(0.0f), mR22(1.0f) {}
        
      /**
      * Construct a 3D rigid body transform. 
      * @param[in] x The x translation component.
      * @param[in] y The y translation component.
      * @param[in] z The z translation component.
      * @param[in] r00 The row = 1, col = 1 rotation component.
      * @param[in] r01 The row = 1, col = 2 rotation component.
      * @param[in] r02 The row = 1, col = 3 rotation component.
      * @param[in] r10 The row = 2, col = 1 rotation component.
      * @param[in] r11 The row = 2, col = 2 rotation component.
      * @param[in] r12 The row = 2, col = 3 rotation component.
      * @param[in] r20 The row = 3, col = 1 rotation component.
      * @param[in] r21 The row = 3, col = 2 rotation component.
      * @param[in] r22 The row = 3, col = 3 rotation component.
      */
      GeneralFrame(float x, float y, float z, float r00, float r01, float r02, float r10, float r11, float r12, float r20, float r21, float r22) :
        mX(x), mY(y), mZ(z), mR00(r00), mR01(r01), mR02(r02), mR10(r10), mR11(r11), mR12(r12), mR20(r20), mR21(r21), mR22(r22) {}

      float mX; /**< The x translation component. */
      float mY; /**< The y translation component. */
      float mZ; /**< The z translation component. */
      float mR00; /**< The row = 1, col = 1 rotation component. */
      float mR01; /**< The row = 1, col = 2 rotation component. */
      float mR02; /**< The row = 1, col = 3 rotation component. */
      float mR10; /**< The row = 2, col = 1 rotation component. */
      float mR11; /**< The row = 2, col = 2 rotation component. */
      float mR12; /**< The row = 2, col = 3 rotation component. */
      float mR20; /**< The row = 3, col = 1 rotation component. */
      float mR21; /**< The row = 3, col = 2 rotation component. */
      float mR22; /**< The row = 3, col = 3 rotation component. */

    };

    /**
    * @struct DenavitHartenbergFrame
    * The reference frame transformation for a kinematic chain when using the Denavit Hartenberg parameters.
    */
    struct DenavitHartenbergFrame{

      /**
      * Default constructor. Sets a, &alpha, d, &theta to zero.
      */
      DenavitHartenbergFrame() : mFrame(0), mJointType(JointTypeEnum::FIXED), mA(0.0f), mAlpha(0.0f), mD(0.0f), mTheta(0.0f){}

      /**
      * Sets up a Denavit Hartenberg Frame.
      * @param[in] frame The frame index.
      * @param[in] joint_type Whether the joint is rotary, prismatic or fixed.
      * @param[in] a The value of a.
      * @param[in] alpha The value of &alpha.
      * @param[in] d The value of d.
      * @param[in] theta The value of &theta.
      */
      DenavitHartenbergFrame(unsigned int frame, JointTypeEnum::Enum joint_type, float a, float alpha, float d, float theta) :
        mFrame(frame), mJointType(joint_type), mA(a), mAlpha(alpha), mD(d), mTheta(theta){ }
      
      unsigned int mFrame; /**< The frame index. */
      JointTypeEnum::Enum mJointType; /**< The joint type. */
      float mA; /**< The value of a. */
      float mAlpha; /**< The value of &alpha. */
      float mD; /**< The value of d. */
      float mTheta; /**< The value of &theta. */

    };

    struct DaVinciKinematicChain {

      DaVinciKinematicChain();

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

    void glhSetIdentity(GLdouble* A);

    void glhMultMatrixRight(const GLdouble* A, GLdouble* B);

    void extendChain(const GeneralFrame& frame, GLdouble* A);

    void glhDenavitHartenberg(const double a, const double alpha, const double d, const double theta, GLdouble *A);

    void extendChain(const DenavitHartenbergFrame& frame, GLdouble* A, float angle);

    void buildKinematicChainPSM1(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, ci::Matrix44f &roll, ci::Matrix44f &wrist_pitch, ci::Matrix44f &grip1, ci::Matrix44f &grip2);

    void buildKinematicChainAtEndPSM1(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, ci::Matrix44f &roll, ci::Matrix44f &wrist_pitch, ci::Matrix44f &grip1, ci::Matrix44f &grip2);

    void buildKinematicChainPSM2(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, ci::Matrix44f &roll, ci::Matrix44f &wrist_pitch, ci::Matrix44f &grip1, ci::Matrix44f &grip2);

    void buildKinematicChainAtEndPSM2(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, ci::Matrix44f &roll, ci::Matrix44f &wrist_pitch, ci::Matrix44f &grip1, ci::Matrix44f &grip2);

    void buildKinematicChainECM1(DaVinciKinematicChain &mDaVinciChain, const ECMData& ecm, ci::Matrix44f &world_to_camera_transform);

    void updateKinematicChain(void);

    bool gluInvertMatrix(const double m[16], double invOut[16]);
  }
}