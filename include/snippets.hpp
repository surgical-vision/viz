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

void buildKinematicChainPSM1(DaVinciKinematicChain &mDaVinciChain, const API_PSM& psm, GLdouble* A, viz::Pose &suj_frames, viz::Pose &frames);

void buildKinematicChainPSM2(DaVinciKinematicChain &mDaVinciChain, API_PSM& psm, GLdouble* A, viz::Pose &suj_frames, viz::Pose &frames);

void buildKinematicChainECM1(DaVinciKinematicChain &mDaVinciChain, const API_ECM& ecm, GLdouble* A, viz::Pose &suj_frames, viz::Pose &frames);

void updateKinematicChain(void);

bool gluInvertMatrix(const double m[16], double invOut[16]);