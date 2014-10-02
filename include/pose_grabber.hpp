#pragma once

#include <cinder/Matrix.h>
#include <fstream>
#include <vector>
#include "davinci.hpp"
#include <boost/shared_ptr.hpp>

namespace viz {


  struct Pose {
  
    Pose() {}
    Pose(ci::Matrix44f &val) { poses_.push_back(std::make_pair<>(val,0.0)); }
    std::vector<std::pair<ci::Matrix44f,double> > poses_;  
    boost::shared_ptr< std::vector<double> > offsets_;
    boost::shared_ptr< std::vector<double> > base_offsets_;

    std::vector<double> suj_dh_vals_;
    std::vector<double> j_dh_vals_;

  };


  class BasePoseGrabber {

  public:
    
    virtual Pose getPose(bool load_new) = 0;
    virtual ~BasePoseGrabber() {};  

  protected:
    void convertFromBouguetPose(const ci::Matrix44f &in_pose, ci::Matrix44f &out_pose);


  };


  class PoseGrabber : public BasePoseGrabber {

  public:
    
    PoseGrabber(const std::string &infile);
    virtual Pose getPose(bool load_new);

  protected:
    std::ifstream ifs_;
    ci::Matrix44f gl_next_pose_;

  };

  class ArticulatedPoseGrabber : public BasePoseGrabber {

  public:
     
    ArticulatedPoseGrabber(const std::string &infile);
    virtual Pose getPose(bool load_new);

  protected:
    std::ifstream ifs_;
    Pose cached_pose_;

  };
  


  class DaVinciPoseGrabber : public BasePoseGrabber {

  public:

    DaVinciPoseGrabber(const std::string &in_suj_file, const std::string &in_j_file, const davinci::DaVinciJoint joint_type);
    virtual Pose getPose(bool load_new);
    void setupOffsets(int n);
    void setOffsets(double a1, double a2, double a3, double a4, double a5, double a6, double a7);
    void setBaseOffsets(double a1, double a2, double a3, double a4, double a5, double a6);
    boost::shared_ptr< std::vector<double> > getOffsets() { return offsets_; }
    boost::shared_ptr< std::vector<double> > getBaseOffsets() { return base_offsets_; }
  
  protected:

    void convertFromDaVinciPose(const ci::Matrix44f &in_pose, ci::Matrix44f &out_pose);
    void ReadDHFromFiles(std::vector<double> &psm_suj_joints, std::vector<double> &psm_joints);

    std::ifstream suj_ifs_;
    std::ifstream j_ifs_;
    
    std::size_t num_suj_joints_;
    std::size_t num_j_joints_;

    davinci::DaVinciKinematicChain chain_;
    davinci::DaVinciJoint target_joint_;

    boost::shared_ptr< std::vector<double> > offsets_; 
    boost::shared_ptr< std::vector<double> > base_offsets_;

    std::vector<double> suj_joints, j_joints;

  };





}