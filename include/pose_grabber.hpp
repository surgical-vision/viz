#pragma once

#include <cinder/Matrix.h>
#include <fstream>
#include <vector>
#include "davinci.hpp"
#include <boost/shared_ptr.hpp>

namespace viz {


  struct Pose {
  
    Pose() {}
    Pose(ci::Matrix44f &val) { poses_.push_back(val); }

    std::vector<ci::Matrix44f> poses_;  
  
    boost::shared_ptr< std::vector<double> > offsets_;

  };


  class BasePoseGrabber {

  public:
    
    BasePoseGrabber() : saving_(false) {}
    virtual Pose getPose(bool load_new) = 0;
    virtual ~BasePoseGrabber() {};
    
    void setSave(bool to_save){
      saving_ = to_save; 
    }

  protected:

    void convertFromBouguetPose(const ci::Matrix44f &in_pose, ci::Matrix44f &out_pose);

    bool saving_;

  };


  class PoseGrabber : public BasePoseGrabber {

  public:
    
    PoseGrabber(const std::string &infile);
    virtual Pose getPose(bool load_new);

  protected:
    std::ifstream ifs_;
    ci::Matrix44f gl_next_pose_;

  };

  


  class DaVinciPoseGrabber : public BasePoseGrabber {

  public:

    DaVinciPoseGrabber(const std::string &in_suj_file, const std::string &in_j_file, const davinci::DaVinciJoint joint_type);
    virtual Pose getPose(bool load_new);
    void setupOffsets(int n);

    boost::shared_ptr< std::vector<double> > getOffsets() { return offsets_; }

  protected:

    void savePoseAsSE3(std::ofstream &ofs, const ci::Matrix44d &camera_pose);
    void savePoseAsSE3AndDH(std::ofstream &ofs, const ci::Matrix44d &camera_pose);
    void convertFromDaVinciPose(const ci::Matrix44f &in_pose, ci::Matrix44f &out_pose);
    void ReadDHFromFiles(std::vector<double> &psm_suj_joints, std::vector<double> &psm_joints);

    std::ifstream suj_ifs_;
    std::ifstream j_ifs_;
    
    std::size_t num_suj_joints_;
    std::size_t num_j_joints_;

    davinci::DaVinciKinematicChain chain_;
    davinci::DaVinciJoint target_joint_;

    boost::shared_ptr< std::vector<double> > offsets_; 
    std::vector<double> suj_joints, j_joints;

  };





}