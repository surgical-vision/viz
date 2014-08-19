#pragma once

#include <string>
#include <boost/scoped_ptr.hpp>
#include <track/model/articulated_model.hpp>
#include "pose_grabber.hpp"

namespace viz {

  class Trackable {


  public:

    void setupPoseGrabber(const std::string &filename);
    void setupDaVinciPoseGrabber(const std::string &suj_file, const std::string &j_file, const std::string &da_vinci_config, davinci::DaVinciJoint joint_type);

  protected:

    boost::scoped_ptr<ttrk::Model> trackable_;
    boost::shared_ptr<BasePoseGrabber> pose_grabber_;

  };




}