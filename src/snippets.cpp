#include "snippets.hpp"
#include <cinder/app/App.h>

/******************************************************************************************************************************/

// OpenGL matrix offsets
#define _00 0
#define _10 1
#define _20 2
#define _30 3
#define _01 4
#define _11 5
#define _21 6
#define _31 7
#define _02 8
#define _12 9
#define _22 10
#define _32 11
#define _03 12
#define _13 13
#define _23 14
#define _33 15

const double SCALE = 1000;//41.8; //1000.0

// Set identity matrix
void glhSetIdentity(GLdouble* A)
{
	A[_00] = 1.0; A[_01] = 0.0; A[_02] = 0.0; A[_03] = 0.0;
	A[_10] = 0.0; A[_11] = 1.0; A[_12] = 0.0; A[_13] = 0.0;
	A[_20] = 0.0; A[_21] = 0.0; A[_22] = 1.0; A[_23] = 0.0;
	A[_30] = 0.0; A[_31] = 0.0; A[_32] = 0.0; A[_33] = 1.0;
}

/******************************************************************************************************************************/

// Matrix multiplication (B = BA)
void glhMultMatrixRight(const GLdouble* A, GLdouble* B)
{
	GLdouble M[16];
	M[_00] = B[_00] * A[_00] + B[_01] * A[_10] + B[_02] * A[_20] + B[_03] * A[_30];
	M[_10] = B[_10] * A[_00] + B[_11] * A[_10] + B[_12] * A[_20] + B[_13] * A[_30];
	M[_20] = B[_20] * A[_00] + B[_21] * A[_10] + B[_22] * A[_20] + B[_23] * A[_30];
	M[_30] = B[_30] * A[_00] + B[_31] * A[_10] + B[_32] * A[_20] + B[_33] * A[_30];
	M[_01] = B[_00] * A[_01] + B[_01] * A[_11] + B[_02] * A[_21] + B[_03] * A[_31];
	M[_11] = B[_10] * A[_01] + B[_11] * A[_11] + B[_12] * A[_21] + B[_13] * A[_31];
	M[_21] = B[_20] * A[_01] + B[_21] * A[_11] + B[_22] * A[_21] + B[_23] * A[_31];
	M[_31] = B[_30] * A[_01] + B[_31] * A[_11] + B[_32] * A[_21] + B[_33] * A[_31];
	M[_02] = B[_00] * A[_02] + B[_01] * A[_12] + B[_02] * A[_22] + B[_03] * A[_32];
	M[_12] = B[_10] * A[_02] + B[_11] * A[_12] + B[_12] * A[_22] + B[_13] * A[_32];
	M[_22] = B[_20] * A[_02] + B[_21] * A[_12] + B[_22] * A[_22] + B[_23] * A[_32];
	M[_32] = B[_30] * A[_02] + B[_31] * A[_12] + B[_32] * A[_22] + B[_33] * A[_32];
	M[_03] = B[_00] * A[_03] + B[_01] * A[_13] + B[_02] * A[_23] + B[_03] * A[_33];
	M[_13] = B[_10] * A[_03] + B[_11] * A[_13] + B[_12] * A[_23] + B[_13] * A[_33];
	M[_23] = B[_20] * A[_03] + B[_21] * A[_13] + B[_22] * A[_23] + B[_23] * A[_33];
	M[_33] = B[_30] * A[_03] + B[_31] * A[_13] + B[_32] * A[_23] + B[_33] * A[_33];
	for(unsigned int i = 0; i < 16; i++)
		B[i] = M[i];
}

/******************************************************************************************************************************/

// Concatenate kinematic chain transformations
void extendChain(const GeneralFrame& frame, GLdouble* A)
{
	GLdouble G[16];

	G[_00] = frame.mR00;
	G[_10] = frame.mR10;
	G[_20] = frame.mR20;
	G[_30] = 0.0;
	G[_01] = frame.mR01;
	G[_11] = frame.mR11;
	G[_21] = frame.mR21;
	G[_31] = 0.0;
	G[_02] = frame.mR02;
	G[_12] = frame.mR12;
	G[_22] = frame.mR22;
	G[_32] = 0.0;
  G[_03] = frame.mX * SCALE;
  G[_13] = frame.mY * SCALE;
  G[_23] = frame.mZ * SCALE;
	G[_33] = 1.0;

	glhMultMatrixRight(G, A);

}

/******************************************************************************************************************************/

// Create Denavit-Hartenberg transformation 
void glhDenavitHartenberg(GLdouble a, GLdouble alpha, GLdouble d, GLdouble theta, GLdouble* A) {
  
  GLdouble sa = sin(alpha);
  GLdouble ca = cos(alpha);
  GLdouble st = sin(theta);
  GLdouble ct = cos(theta);

  A[_00] = ct;
  A[_10] = ca * st;
  A[_20] = sa * st;
  A[_30] = 0.0;
  A[_01] = -st;
  A[_11] = ca * ct;
  A[_21] = sa * ct;
  A[_31] = 0.0;
  A[_02] = 0.0;
  A[_12] = -sa;
  A[_22] = ca;
  A[_32] = 0.0;
  A[_03] = a;
  A[_13] = -sa * d;
  A[_23] = ca * d;
  A[_33] = 1.0;

}


// Concatenate kinematic chain transformations
void extendChain(const DenavitHartenbergFrame& frame, GLdouble* A, float angle = 0.0)
{
	GLdouble DH[16];

	switch(frame.mJointType)
	{
		case JointTypeEnum::FIXED:
      glhDenavitHartenberg(frame.mA * SCALE, frame.mAlpha, frame.mD * SCALE, frame.mTheta, DH);
		  break;

		case JointTypeEnum::ROTARY:
      glhDenavitHartenberg(frame.mA * SCALE, frame.mAlpha, frame.mD * SCALE, frame.mTheta + angle, DH);
		  break;

		case JointTypeEnum::PRISMATIC:
      glhDenavitHartenberg(frame.mA * SCALE, frame.mAlpha, (frame.mD + angle) * SCALE, frame.mTheta, DH);
		  break;
	}

	glhMultMatrixRight(DH, A);
}

/******************************************************************************************************************************/



/******************************************************************************************************************************/

// Concatenate chain transformations
void buildKinematicChainPSM2(DaVinciKinematicChain &mDaVinciChain, API_PSM& psm, GLdouble* A, viz::Pose &suj_frames, viz::Pose &frames)
{
  glhSetIdentity(A);
  extendChain(mDaVinciChain.mWorldOriginSUJ2Origin[0], A);
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[0], A, psm.sj_joint_angles[0] + frames.base_offsets_->operator[](0));
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[1], A, psm.sj_joint_angles[1] + frames.base_offsets_->operator[](1));
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[2], A, psm.sj_joint_angles[2] + frames.base_offsets_->operator[](2));
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[3], A, psm.sj_joint_angles[3] + frames.base_offsets_->operator[](3));
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[4], A, psm.sj_joint_angles[4] + frames.base_offsets_->operator[](4));
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[5], A, psm.sj_joint_angles[5] + frames.base_offsets_->operator[](5));
 
  extendChain(mDaVinciChain.mSUJ2TipPSM2Origin[0], A);
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[0], A, psm.jnt_pos[0] + frames.offsets_->operator[](0));
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[1], A, psm.jnt_pos[1] + frames.offsets_->operator[](1));
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[2], A, psm.jnt_pos[2] + frames.offsets_->operator[](2));

  //add the instrument roll - this corresponds to a ref frame attached to the wrist end of the shaft around which the instrument rolls
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[3], A, psm.jnt_pos[3] + frames.offsets_->operator[](3));
  ci::Matrix44d roll(A);
  frames.poses_.push_back(std::make_pair<>(roll, psm.jnt_pos[3] + frames.offsets_->operator[](3)));

  //add the instrument wrist pitch 
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[4], A, psm.jnt_pos[4] + frames.offsets_->operator[](4));
  ci::Matrix44d wrist_pitch(A);
  frames.poses_.push_back(std::make_pair<>(wrist_pitch, psm.jnt_pos[4] + frames.offsets_->operator[](4)));

  //don't actually care about wrist yaw as the clasper pose 'contains' this information
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[5], A, psm.jnt_pos[5] + frames.offsets_->operator[](5));

  //transform into the clasper reference frame
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[6], A, 0);  //no dh param here as this just points in the direction of the instrument head
  //frames.poses_.push_back(std::make_pair<>(ci::Matrix44d(A), psm.jnt_pos[5] + frames.offsets_->operator[](5)));

  //rotate the instrument claspers around the clasper axis 
  ci::Matrix44d grip1(A);
  ci::Matrix44d grip2(A);

  //this is the angle between the claspers, so each clasper rotates 0.5*angle away from the center point
  double val = psm.jnt_pos[6] + frames.offsets_->operator[](6);
  grip1.rotate(ci::Vec3d(0, 1, 0), 0.5*val);
  frames.poses_.push_back(std::make_pair(grip1, 0.5*val));

  grip2.rotate(ci::Vec3d(0, 0, 1), M_PI);
  grip2.rotate(ci::Vec3d(0, 1, 0), 0.5*val);
  frames.poses_.push_back(std::make_pair(grip2, 0.5*val));

}

/******************************************************************************************************************************/

// Concatenate chain transformations
void buildKinematicChainECM1(DaVinciKinematicChain &mDaVinciChain, const API_ECM& ecm, GLdouble* A, viz::Pose &suj_frames, viz::Pose &frames)
{
  glhSetIdentity(A);
  extendChain(mDaVinciChain.mWorldOriginSUJ3Origin[0], A);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[0], A, ecm.sj_joint_angles[0]);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[1], A, ecm.sj_joint_angles[1]);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[2], A, ecm.sj_joint_angles[2]);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[3], A, ecm.sj_joint_angles[3]);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[4], A);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[5], A);
  extendChain(mDaVinciChain.mSUJ3TipECM1Origin[0], A);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[0], A, ecm.jnt_pos[0]);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[1], A, ecm.jnt_pos[1]);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[2], A, ecm.jnt_pos[2]);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[3], A, ecm.jnt_pos[3]);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[4], A);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[5], A);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[6], A);
  frames.poses_.push_back(std::make_pair(cinder::Matrix44d(A), 0));
}


// Concatenate chain transformations
void buildKinematicChainPSM1(DaVinciKinematicChain &mDaVinciChain, const API_PSM& psm, GLdouble* A, viz::Pose &suj_frames, viz::Pose &frames)
{

  glhSetIdentity(A);

  extendChain(mDaVinciChain.mWorldOriginSUJ1Origin[0], A);
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[0], A, psm.sj_joint_angles[0] + frames.base_offsets_->operator[](0));
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[1], A, psm.sj_joint_angles[1] + frames.base_offsets_->operator[](1));
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[2], A, psm.sj_joint_angles[2] + frames.base_offsets_->operator[](2));
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[3], A, psm.sj_joint_angles[3] + frames.base_offsets_->operator[](3));
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[4], A, psm.sj_joint_angles[4] + frames.base_offsets_->operator[](4));
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[5], A, psm.sj_joint_angles[5] + frames.base_offsets_->operator[](5));

  extendChain(mDaVinciChain.mSUJ1TipPSM1Origin[0], A);
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[0], A, psm.jnt_pos[0] + frames.offsets_->operator[](0));
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[1], A, psm.jnt_pos[1] + frames.offsets_->operator[](1));
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[2], A, psm.jnt_pos[2] + frames.offsets_->operator[](2));
  
  //add the instrument roll - this corresponds to a ref frame attached to the wrist end of the shaft around which the instrument rolls
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[3], A, psm.jnt_pos[3] + frames.offsets_->operator[](3));
  ci::Matrix44d roll(A);
  frames.poses_.push_back(std::make_pair<>(roll, psm.jnt_pos[3] + frames.offsets_->operator[](3)));
  

  //add the instrument wrist pitch 
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[4], A, psm.jnt_pos[4] + frames.offsets_->operator[](4));
  ci::Matrix44d wrist_pitch(A);
  frames.poses_.push_back(std::make_pair<>(wrist_pitch, psm.jnt_pos[4] + frames.offsets_->operator[](4)));

  //don't actually care about wrist yaw as the clasper pose 'contains' this information
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[5], A, psm.jnt_pos[5] + frames.offsets_->operator[](5));
  ci::Matrix44d wrist_yaw(A);

  //transform into the clasper reference frame
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[6], A, 0);  //no dh param here as this just points in the direction of the instrument head
  //frames.poses_.push_back(std::make_pair<>(ci::Matrix44d(A), psm.jnt_pos[5] + frames.offsets_->operator[](5)));

  //rotate the instrument claspers around the clasper axis 
  ci::Matrix44d grip1(A);
  ci::Matrix44d grip2(A);

  //this is the angle between the claspers, so each clasper rotates 0.5*angle away from the center point
  double val = psm.jnt_pos[6] + frames.offsets_->operator[](6);
  grip1.rotate(ci::Vec3d(0, 1, 0), 0.5*val);
  frames.poses_.push_back(std::make_pair(grip1, 0.5*val));

  grip2.rotate(ci::Vec3d(0, 0, 1), M_PI);
  grip2.rotate(ci::Vec3d(0, 1, 0), 0.5*val);
  frames.poses_.push_back(std::make_pair(grip2,0.5*val)); 
    
}

