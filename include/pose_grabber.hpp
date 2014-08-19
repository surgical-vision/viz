#pragma once

#include <cinder/Matrix.h>
#include <fstream>
#include <vector>

namespace viz {


  struct Pose {
  
    Pose(ci::Matrix44f &val) { poses_.push_back(val); }

    std::vector<ci::Matrix44f> poses_;  
  
  };


  class BasePoseGrabber {

  public:
    
    virtual Pose getNextPose() = 0;
    virtual ~BasePoseGrabber() {};

  protected:

    void convertFromBouguetPose(const ci::Matrix44f &in_pose, ci::Matrix44f &out_pose);

  };


  class PoseGrabber : public BasePoseGrabber {

  public:
    
    PoseGrabber(const std::string &infile);
    virtual Pose getNextPose();

  protected:
    std::ifstream ifs_;

  };



  class DaVinciPoseGrabber : public BasePoseGrabber {

  public:

    DaVinciPoseGrabber(const std::string &in_suj_file, const std::string &in_j_file);
    virtual Pose getNextPose();

  protected:

    std::ifstream suj_ifs_;
    std::ifstream j_ifs_;
  
  };





}