#pragma once

#include <cinder/Matrix.h>
#include <fstream>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "davinci.hpp"
#include "../include/config_reader.hpp"
#include "../include/model.hpp"

namespace viz {


  /**
  * @class BasePoseGrabber
  * @brief An abstract class to represent a pose grabbing interface.
  * This class specifies the interface to reading pose values from files and then rendering its model at that pose.
  * Also capable for saving the pose estimates to a file in a different format (i.e. DH to SE3 etc.)
  */
  class BasePoseGrabber {

  public:

    /**
    * Default constructor. Sets the do_draw_ flag to false so that nothing gets drawn until we've successfully read a pose value from a pose file.
    */
    BasePoseGrabber() : do_draw_(false) {}
    
    /**
    * Load the next pose value from the file. This changes the class' internal representation of its pose when it renders the model.
    * @param[in] no_reload Just refresh the internal representation of pose without reading anything from the pose file. This is useful if the pose can be manually modified within the
    * UI to account for a rigid offset.
    */
    virtual void LoadPose(const bool no_reload) = 0;
    
    /**
    * Return the current pose estimate.
    * @return Return the current pose estimate.
    */
    virtual ci::Matrix44f GetPose() = 0;

    /**
    * Destructor.
    */
    virtual ~BasePoseGrabber() {};  

    /**
    * Writes the current estimate of pose to a string.
    * @return The formatted version of the current pose.
    */
    virtual std::string writePoseToString() const = 0;

    /**
    * Renders the model to the currently bound framebuffer. Assumes OpenGL context is available on current thread.
    */
    virtual void Draw() const = 0;

    /**
    * Write this pose grabber's computed pose to an output file with a camera pose transform if this pose should be saved in
    * camera coordinates.
    * @return A formatted representation of the object's pose.
    */
    virtual std::string writePoseToString(const ci::Matrix44f &camera_pose) const = 0;

    /**
    * Get the poses from the previous frames to draw past trajectories.
    * @return A vector of all previous frame's poses.
    */
    std::vector<ci::Matrix44f> &History() { return reference_frame_tracks_;  }

    /**
    * Get the poses from the previous frames to draw past trajectories.
    * @return A vector of all previous frame's poses.
    */
    const std::vector<ci::Matrix44f> &History() const { return reference_frame_tracks_; }

  protected:
    
    void checkSelfName(const std::string &test_name) const { if (test_name != self_name_) throw std::runtime_error(""); }

    /**
    * Is this still needed?
    * 
    */
    void convertFromBouguetPose(const ci::Matrix44f &in_pose, ci::Matrix44f &out_pose);

    bool do_draw_; /**< Flag set to false when there are no pose value left to draw the object. */

    std::vector<ci::Matrix44f> reference_frame_tracks_; /**< Keeps track of previous SE3s to represent the model for plotting trajectories across 3D space. For articulated bodies this should be the 'global' pose of the object. */

    std::string self_name_;

  };

  /**
  * @class PoseGrabber
  * @brief A regular SE3 pose grabber.
  * This class reads SE3 rigid body transforms from a pose file and draws the model at that pose in the camera coordinate frame.
  */
  class PoseGrabber : public BasePoseGrabber {

  public:
    
    /**
    * Load a pose grabber from a config file. This file contains the model coordinate file (if applicable) and pose file.
    */
    explicit PoseGrabber(const ConfigReader &reader);

    /**
    * Load the next SE3 pose transform from the file. This changes the class' internal representation of its pose when it renders the model.
    * @param[in] no_reload Just refresh the internal representation of pose without reading anything from the pose file. This is useful if the pose can be manually modified within the
    * UI to account for a rigid offset.
    */
    virtual void LoadPose(const bool no_reload);
    
    /**
    * Return the current pose estimate.
    * @return Return the current pose estimate.
    */
    virtual ci::Matrix44f GetPose() { return cached_model_pose_; }
    
    /**
    * Renders the model to the currently bound framebuffer. Assumes OpenGL context is available on current thread.
    */
    virtual void Draw() const { model_.Draw(); }

    /**
    * Write this pose grabber's computed pose to an output file.
    * @return A formatted representation of the object's pose.
    */
    virtual std::string writePoseToString() const;

    /**
    * Write this pose grabber's computed pose to an output file with a camera pose transform if this pose should be saved in
    * camera coordinates.
    * @return A formatted representation of the object's pose.
    */
    virtual std::string writePoseToString(const ci::Matrix44f &camera_pose) const;

  protected:
    
    std::ifstream ifs_; /**< The file stream containing the SE3 transforms for each frame. */
    
    Model model_; /**< The Model to draw for the object. May be empty if for example the PoseGrabber represents a camera. */

    ci::Matrix44f cached_model_pose_; /**< Maintain a cache of object pose so that model can be refreshed without reloading. */

  };

  /**
  * @class BaseDaVinciPoseGrabber
  * @brief An abstract class to represent a manipulator on a da Vinci robot.
  * Overridden to handle different input methods for the manipulator pose computation. 
  * Also capable for saving the pose estimates to a file in a different format (i.e. DH to SE3 etc.)
  */
  class BaseDaVinciPoseGrabber : public BasePoseGrabber {

  public:

    /**
    * Construct a base class da vinci pose grabber. This really just loads the model.
    * @param[in] reader The configuration file.
    */
    BaseDaVinciPoseGrabber(const ConfigReader &reader);

    /**
    * Load the next pose value from the file. This changes the class' internal representation of its pose when it renders the model.
    * @param[in] no_reload Just refresh the internal representation of pose without reading anything from the pose file. This is useful if the pose can be manually modified within the
    * UI to account for a rigid offset.
    */
    virtual void LoadPose(const bool no_reload) = 0;

    /**
    * Renders the model to the currently bound framebuffer. Assumes OpenGL context is available on current thread.
    */
    virtual void Draw() const { model_.Draw(); }

  protected:

    /**
    * Possibly not needed?
    */
    void convertFromDaVinciPose(const ci::Matrix44f &in_pose, ci::Matrix44f &out_pose);

    davinci::DaVinciKinematicChain chain_; /**< */
    davinci::DaVinciJoint target_joint_; /**< */
    DaVinciInstrument model_; /**< */

  };

  /**
  * @class DHDaVinciPoseGrabber
  * @brief A class to represent a da vinci robot manipulator which has been tracked in a camera coordinate frame.
  * Normal da Vinci pose computation is done in world coordinates with DH parameters. If we have tracked a da Vinci instrument
  * in the camera reference frame (so have an SE3 to its body frame) with the DH parameters estimating each component of the endowrist
  * articulation then this class can read and visualize this pose.
  */
  class DHDaVinciPoseGrabber : public BaseDaVinciPoseGrabber {

  public:

    /**
    * Construct a DH parameter da Vinci manipulator from a configuration file.
    * @param[in] reader A ConfigReader instance which has been initialized from a config file.
    */
    DHDaVinciPoseGrabber(const ConfigReader &reader);

    /**
    * Load a set of DH parameters from a file and set up the manipulator transforms using the DH chain.
    * @param[in] no_reloa d Just refresh the internal representation of pose without reading anything from the pose file. This is useful if the pose can be manually modified within the
    * UI to account for a rigid offset.
    */
    virtual void LoadPose(const bool no_reload);

    /**
    * Return the current pose estimate.
    * @return Return the current pose estimate.
    */
    virtual ci::Matrix44f GetPose();

    /**
    * As the DH parameters collected from the da Vinci joint encoders have some fixed offsets, the offset vectors can be used
    * to add a fixed value to each parameter to ensure that the the manipulator aligns correctly with the camera view.
    * @return The offsets vector for the arm so these can be update from the UI.
    */
    std::vector<double> &getArmOffsets() { return arm_offsets_; }

    /**
    * As the DH parameters collected from the da Vinci joint encoders have some fixed offsets, the offset vectors can be used
    * to add a fixed value to each parameter to ensure that the the manipulator aligns correctly with the camera view.
    * @return The offsets vector for the base arm (setup joints) so these can be update from the UI.
    */
    std::vector<double> &getBaseOffsets() { return base_offsets_; }

    /**
    * Write this pose grabber's computed pose to an output file.
    * @return A formatted representation of the object's pose.
    */
    virtual std::string writePoseToString() const;

    /**
    * Write this pose grabber's computed pose to an output file with a camera pose transform if this pose should be saved in
    * camera coordinates.
    * @return A formatted representation of the object's pose.
    */
    virtual std::string writePoseToString(const ci::Matrix44f &camera_pose) const;

  protected:

    /**
    * Read the DH values from the files and store them in the vectors.
    * @param[out] psm_base_joints The base 
    * @param[out] psm_arm_joints The arm joints
    * @return Whether the read was successful
    */
    bool ReadDHFromFiles(std::vector<double> &psm_base_joints, std::vector<double> &psm_arm_joints);

    std::ifstream base_ifs_; /**< The input file stream for the base joint values. */
    std::ifstream arm_ifs_; /**< The input file stream for the arm joint values. */

    std::vector<double> arm_offsets_; /**< Arm offset values. */
    std::vector<double> base_offsets_; /**< Base offset values. */
    
    std::vector<double> arm_joints_; /**< Maintain a cache of arm joint values so that arm can be refreshed without reloading. */
    std::vector<double> base_joints_; /**< Maintain a cache of base joint values so that arm can be refreshed without reloading. */
    
    std::size_t num_base_joints_; /**< The number of joints in the robot base arm (setup joints). */
    std::size_t num_arm_joints_; /**< The number of joints in the robot arm. */

  };
  
  /**
  * @class SE3DaVinciPoseGrabber
  * @brief A class to represent a da vinci robot manipulator which has been tracked in a camera coordinate frame.
  * Normal da Vinci pose computation is done in world coordinates with DH parameters. If we have tracked a da Vinci instrument
  * in the camera reference frame (so have an SE3 to its body frame) with the DH parameters estimating each component of the endowrist
  * articulation then this class can read and visualize this pose.
  */
  class SE3DaVinciPoseGrabber : public BaseDaVinciPoseGrabber {

  public:

    /**
    * Construct from a configuration file.
    * @param[in] reader The configuration file reader containing the data about the instrument.
    * @param[in] arm The actual arm configuration to use (PSM1, PSM2, ECM...).
    */
    SE3DaVinciPoseGrabber(const ConfigReader &reader);

    /**
    * Override the pose loader method which normally accepts DH parameters to accept SE3 + DH parameters.
    * @param[in] no_reload Just refresh the internal representation of pose without reading anything from the pose file. This is useful if the pose can be manually modified within the
    * UI to account for a rigid offset.
    */
    virtual void LoadPose(const bool no_reload);

    /**
    * Return the current pose estimate.
    * @return Return the current pose estimate.
    */
    virtual ci::Matrix44f GetPose() { return shaft_pose_; }

    /**
    * Write this pose grabber's computed pose to an output file. 
    * @return A formatted representation of the object's pose.
    */
    virtual std::string writePoseToString() const;

    /**
    * Write this pose grabber's computed pose to an output file with a camera pose transform if this pose should be saved in 
    * camera coordinates.
    * @return A formatted representation of the object's pose.
    */
    virtual std::string writePoseToString(const ci::Matrix44f &camera_pose) const;

  protected:

    std::size_t num_wrist_joints_; /**< Number of joints in the wrist of the instrument. */
    std::ifstream ifs_; /**< The file to read the DH and SE3 values from. */

    ci::Matrix44f shaft_pose_; /**< Maintain a cache of shaft pose value so that model can be refreshed without reloading. */
    std::vector<double> wrist_dh_params_; /**< Maintain a  */

  };




}