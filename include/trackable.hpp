#pragma once

#include <string>
#include <boost/scoped_ptr.hpp>

#include "pose_grabber.hpp"

namespace viz {

  /*
   * Combines a pose grabber with a mesh/model
   */

  /*
  class Trackable {


  public:

    void setupDaVinciModel(const std::string &filename);
    void setupPoseGrabber(const std::string &filename, const std::string &dh_out_file, const std::string &se3_out_file);
    void setupPoseGrabber(const std::string &filename);
    void setupArticulatedPoseGrabber(const std::string &filename);

    void setupDaVinciPoseGrabber(const std::string &suj_file, const std::string &j_file, const std::string &se3_and_dh_out_file, const std::string &suj_dh_out_file, const std::string &j_dh_out_file, const std::string &se3_from_cam_out_file, const std::string &se3_from_world_out_file, const std::string &da_vinci_config, davinci::DaVinciJoint joint_type);
    void setupDaVinciPoseGrabber(const std::string &suj_file, const std::string &j_file, const std::string &se3_and_dh_out_file, const std::string &suj_dh_out_file, const std::string &j_dh_out_file, const std::string &se3_from_cam_out_file, const std::string &se3_from_world_out_file, davinci::DaVinciJoint joint_type);
    
    //void setupDaVinciPoseGrabber(const std::string &suj_file, const std::string &j_file, const std::string &dh_out_file, const std::string &se3_from_cam_out_file, const std::string &se3_from_world_out_file, const std::string &da_vinci_config, davinci::DaVinciJoint joint_type);
    //void setupDaVinciPoseGrabber(const std::string &suj_file, const std::string &j_file, const std::string &dh_out_file, const std::string &se3_from_cam_out_file, const std::string &se3_from_world_out_file, davinci::DaVinciJoint joint_type);
    void getDaVinciPose(bool load_new) { pose_ = getDVPoseGrabber()->getPose(load_new); }
    void getStandardPose(bool load_new) { pose_ = getPoseGrabber()->getPose(load_new); }
    void getNextPose(bool load_new) { pose_ = pose_grabber_->getPose(load_new); }
    Pose getPose() { return pose_; }

    boost::shared_ptr<ttrk::Model> getTrackable() { return trackable_; }
    boost::shared_ptr<ttrk::IntuitiveSurgicalLND> getDaVinciTrackable() { return boost::dynamic_pointer_cast<ttrk::IntuitiveSurgicalLND>(trackable_); }
    boost::shared_ptr<DaVinciPoseGrabber> getDVPoseGrabber() { return boost::dynamic_pointer_cast<DaVinciPoseGrabber>(pose_grabber_); }
    boost::shared_ptr<PoseGrabber> getPoseGrabber() { return boost::dynamic_pointer_cast<PoseGrabber>(pose_grabber_); }
    void setupModel(boost::shared_ptr<ttrk::Model> model);

    std::ofstream &getSe3FromCamStream() { return se3_from_cam_ofs_; }
    std::ofstream &getSe3FromWorldStream() { return se3_from_world_ofs_; }
    std::ofstream &getSe3AndDHStream() { return dh_and_se3_ofs_; }
    std::ofstream &getSujDHStream() { return suj_dh_ofs_; }
    std::ofstream &getJDHStream() { return j_dh_ofs_; }

  protected:

    boost::shared_ptr<ttrk::Model> trackable_;
    boost::shared_ptr<BasePoseGrabber> pose_grabber_;
    Pose pose_;

    std::ofstream dh_and_se3_ofs_;
    std::ofstream suj_dh_ofs_;
    std::ofstream j_dh_ofs_;
    std::ofstream se3_from_cam_ofs_;
    std::ofstream se3_from_world_ofs_;
    

  };
  */



}