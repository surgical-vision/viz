/*
Modified from original by Philip Pratt, Imperial College London
*/


#include "api_stream.h"
#include "pose_grabber.hpp"
#include "davinci.hpp"
#include "camera.hpp"

using namespace viz::davinci;

void glhSetIdentity(GLdouble* A);

void glhMultMatrixRight(const GLdouble* A, GLdouble* B);

void extendChain(const GeneralFrame& frame, GLdouble* A);

void glhDenavitHartenberg(const double a, const double alpha, const double d, const double theta, GLdouble *A);

void extendChain(const DenavitHartenbergFrame& frame, GLdouble* A, float angle);

void buildKinematicChainPSM1(DaVinciKinematicChain &mDaVinciChain, const API_PSM& psm, ci::Matrix44f &roll, ci::Matrix44f &wrist_pitch, ci::Matrix44f &grip1, ci::Matrix44f &grip2);
void buildKinematicChainAtEndPSM1(DaVinciKinematicChain &mDaVinciChain, const API_PSM& psm, ci::Matrix44f &roll, ci::Matrix44f &wrist_pitch, ci::Matrix44f &grip1, ci::Matrix44f &grip2);

void buildKinematicChainPSM2(DaVinciKinematicChain &mDaVinciChain, const API_PSM& psm, ci::Matrix44f &roll, ci::Matrix44f &wrist_pitch, ci::Matrix44f &grip1, ci::Matrix44f &grip2);
void buildKinematicChainAtEndPSM2(DaVinciKinematicChain &mDaVinciChain, const API_PSM& psm, ci::Matrix44f &roll, ci::Matrix44f &wrist_pitch, ci::Matrix44f &grip1, ci::Matrix44f &grip2);

void buildKinematicChainECM1(DaVinciKinematicChain &mDaVinciChain, const API_ECM& ecm, ci::Matrix44f &world_to_camera_transform);

void updateKinematicChain(void);

bool gluInvertMatrix(const double m[16], double invOut[16]);