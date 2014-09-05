#pragma once

#include <string>
#include <boost/scoped_ptr.hpp>
#include <track/model/articulated_model.hpp>
#include "pose_grabber.hpp"

namespace viz {

  /*
   * Combines a pose grabber with a mesh/model
   */

  class Trackable {


  public:

    void setupPoseGrabber(const std::string &filename, const std::string &dh_out_file, const std::string &se3_out_file);
    void setupDaVinciPoseGrabber(const std::string &suj_file, const std::string &j_file, const std::string &dh_out_file, const std::string &se3_out_file, const std::string &da_vinci_config, davinci::DaVinciJoint joint_type);
    void getDaVinciPose(bool load_new) { pose_ = getDVPoseGrabber()->getPose(load_new); }
    void getStandardPose(bool load_new) { pose_ = getPoseGrabber()->getPose(load_new); }
    
    Pose getPose() { return pose_; }

    boost::shared_ptr<ttrk::IntuitiveSurgicalLND> getDaVinciTrackable() { return boost::dynamic_pointer_cast<ttrk::IntuitiveSurgicalLND>(trackable_); }
    boost::shared_ptr<DaVinciPoseGrabber> getDVPoseGrabber() { return boost::dynamic_pointer_cast<DaVinciPoseGrabber>(pose_grabber_); }
    boost::shared_ptr<PoseGrabber> getPoseGrabber() { return boost::dynamic_pointer_cast<PoseGrabber>(pose_grabber_); }
    
    std::ofstream &getSe3Stream() { return se3_ofs_; }
    std::ofstream &getDHStream() { return dh_ofs_; }

  protected:

    boost::shared_ptr<ttrk::Model> trackable_;
    boost::shared_ptr<BasePoseGrabber> pose_grabber_;
    Pose pose_;

    std::ofstream se3_ofs_;
    std::ofstream dh_ofs_;

  };




}