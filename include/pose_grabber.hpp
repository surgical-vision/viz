#pragma once

/**

viz - A robotics visualizer specialized for the da Vinci robotic system.
Copyright (C) 2014 Max Allan

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

**/

#include <cinder/app/App.h>
#include <cinder/Matrix.h>
#include <cinder/params/Params.h>
#include <fstream>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "davinci.hpp"
#include "config_reader.hpp"
#include "model.hpp"
#include "camera.hpp"

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
    * @param[in] output_dir The output directory to save pose files.
    */
    BasePoseGrabber(const std::string &output_dir);
    
    /**
    * Load the next pose value from the file. This changes the class' internal representation of its pose when it renders the model.
    * @param[in] no_reload Just refresh the internal representation of pose without reading anything from the pose file. This is useful if the pose can be manually modified within the UI to account for a rigid offset.
    * @return The success of the load (false if for instance there are no poses left to read from the file).
    */
    virtual bool LoadPose(const bool no_reload) = 0;
    
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
    virtual void WritePoseToStream() = 0;

    virtual void WritePoseToStream(const ci::Matrix44f &camera_pose) = 0;
    
    std::string WriteSE3ToString(const ci::Matrix44f &mat);

    /**
    * Renders the model to the currently bound framebuffer. Assumes OpenGL context is available on current thread.
    */
    virtual void Draw() const = 0;

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

    ci::params::InterfaceGlRef ParamModifier() { return param_modifier_; }

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

    std::string save_dir_;

    ci::params::InterfaceGlRef param_modifier_;

    static size_t grabber_num_id_;

  };

  /**
  * @class PoseGrabber
  * @brief A regular SE3 pose grabber.
  * This class reads SE3 rigid body transforms from a pose file and draws the model at that pose in the camera coordinate frame.
  */
  class PoseGrabber : public BasePoseGrabber {

  public:
    
    PoseGrabber(const std::string &output_dir) : BasePoseGrabber(output_dir) {}

    /**
    * Load a pose grabber from a config file. This file contains the model coordinate file (if applicable) and pose file.
    */
    PoseGrabber(const ConfigReader &reader, const std::string &output_dir);

    /**
    * Load the next SE3 pose transform from the file. This changes the class' internal representation of its pose when it renders the model.
    * @param[in] no_reload Just refresh the internal representation of pose without reading anything from the pose file. This is useful if the pose can be manually modified within the UI to account for a rigid offset.
    * @return The success of the load (false if for instance there are no poses left to read from the file).
    */
    virtual bool LoadPose(const bool no_reload);
    
    virtual void WritePoseToStream();

    virtual void WritePoseToStream(const ci::Matrix44f &camera_pose);

    /**
    * Return the current pose estimate.
    * @return Return the current pose estimate.
    */
    virtual ci::Matrix44f GetPose() { return cached_model_pose_; }
    
    /**
    * Renders the model to the currently bound framebuffer. Assumes OpenGL context is available on current thread.
    */
    virtual void Draw() const { model_.Draw(); }

    virtual ~PoseGrabber() { if (ifs_.is_open()) ifs_.close(); if (ofs_.is_open()) ofs_.close(); }

  protected:
    
    std::ifstream ifs_; /**< The file stream containing the SE3 transforms for each frame. */
    
    std::ofstream ofs_; /**< The file stream to save the output SE£ transforms (as they may have been modified in the UI or given relative to another reference frame). */
    std::string ofs_file_; /**< The actual file to write to, this allows delayed opening. */

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
    BaseDaVinciPoseGrabber(const ConfigReader &reader, const std::string &output_dir);

    /**
    * Load the next pose value from the file. This changes the class' internal representation of its pose when it renders the model.
    * @param[in] update_as_new Refresh the internal representation of pose by reading a new value from the pose file. This is useful if the pose can be manually modified within the UI to account for a rigid offset.
    * @return The success of the load (false if for instance there are no poses left to read from the file).
    */
    virtual bool LoadPose(const bool update_as_new) = 0;

    /**
    * Renders the model to the currently bound framebuffer. Assumes OpenGL context is available on current thread.
    */
    virtual void Draw() const { model_.Draw(); }

  protected:
    virtual void SetOffsetsToNull() = 0;

    virtual void SetupOffsets(const std::string &base_offsets, const std::string &arm_offsets) = 0;


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
    DHDaVinciPoseGrabber(const ConfigReader &reader, const std::string &output_dir);

    /**
    * Load a set of DH parameters from a file and set up the manipulator transforms using the DH chain.
    * @param[in] update_as_new Refresh the internal representation of pose by reading a new value from the pose file. This is useful if the pose can be manually modified within the UI to account for a rigid offset.
    * @return The success of the load (false if for instance there are no poses left to read from the file).
    */
    virtual bool LoadPose(const bool update_as_new);

    /**
    * Return the current pose estimate.
    * @return Return the current pose estimate.
    */
    virtual ci::Matrix44f GetPose();

    virtual void WritePoseToStream();

    virtual void WritePoseToStream(const ci::Matrix44f &camera_pose);

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

    virtual ~DHDaVinciPoseGrabber() { if (base_ifs_.is_open()) base_ifs_.close(); if (arm_ifs_.is_open()) arm_ifs_.close(); if (base_ofs_.is_open()) base_ofs_.close(); if (arm_ofs_.is_open()) arm_ofs_.close(); if (se3_ofs_.is_open()) se3_ofs_.close(); }

    void DrawBody();
    void DrawHead();
    void GetModelPose(ci::Matrix44f &head, ci::Matrix44f &clasper_left, ci::Matrix44f &clasper_right);


    virtual void SetOffsetsToNull() override;


  protected:

    /**
    * Read the DH values from the files and store them in the vectors.
    * @param[in] base_offsets The default base offsets to start with (if we've computed them before and want to start playing around with a better estimate.
    * @param[in] arm_offsets The default arm offsets to start with.
    */
    virtual void SetupOffsets(const std::string &base_offsets, const std::string &arm_offsets) override;

    /**
    * Read the DH values from the files and store them in the vectors.
    * @param[out] psm_base_joints The base 
    * @param[out] psm_arm_joints The arm joints
    * @return Whether the read was successful
    */
    bool ReadDHFromFiles(std::vector<double> &psm_base_joints, std::vector<double> &psm_arm_joints);

    std::ifstream base_ifs_; /**< The input file stream for the base joint values. */
    std::ifstream arm_ifs_; /**< The input file stream for the arm joint values. */

    std::ofstream base_ofs_; /**< The output file to save the base DH parameters as they may have been modified by the UI */
    std::ofstream arm_ofs_; /**< The output file to save the arm DH parameters as they may have been modified by the UI */
    std::ofstream se3_ofs_; /**< The output file to save the SE3 parameters as they may have been modified by the UI */
    
    std::string base_ofs_file_; /**< The actual base DH file to write to, this allows delayed opening. */
    std::string arm_ofs_file_; /**< The actual arm DH file to write to, this allows delayed opening. */
    std::string se3_ofs_file_; /**< The actual SE3 file to write to, this allows delayed opening. */

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
    * @param[in] output_dir The output directory where this session is storing files.
    */
    SE3DaVinciPoseGrabber(const ConfigReader &reader, const std::string &output_dir, bool check_type = true);

    virtual void WritePoseToStream();

    virtual void WritePoseToStream(const ci::Matrix44f &camera_pose);

    /**
    * Override the pose loader method which normally accepts DH parameters to accept SE3 + DH parameters.
    * @param[in] update_as_new Refresh the internal representation of pose by reading a new value from the pose file. This is useful if the pose can be manually modified within the UI to account for a rigid offset.
    * @return The success of the load (false if for instance there are no poses left to read from the file).
    */
    virtual bool LoadPose(const bool update_as_new);

    /**
    * Return the current pose estimate.
    * @return Return the current pose estimate.
    */
    virtual ci::Matrix44f GetPose() { return shaft_pose_; }

    ~SE3DaVinciPoseGrabber() { if (ifs_.is_open()) ifs_.close(); if (ofs_.is_open()) ofs_.close(); }

    void DrawBody();
    void DrawHead();
    void GetModelPose(ci::Matrix44f &head, ci::Matrix44f &clasper_left, ci::Matrix44f &clasper_right);
    
    /**
    * Read the DH values from the files and store them in the vectors.
    * @param[in] base_offsets The default base offsets to start with (if we've computed them before and want to start playing around with a better estimate.
    * @param[in] arm_offsets The default arm offsets to start with.
    */
    virtual void SetupOffsets(const std::string &base_offsets, const std::string &arm_offsets) override;


    void GetSubWindowCoordinates(const viz::Camera &camera, std::array<ci::Vec2i, 4> &rectangle, cv::Mat &affine_transform);


  protected:

    enum LoadType {QUATERNION, MATRIX, EULER};
    LoadType rotation_type_;

    void LoadPoseAsQuaternion();
    void LoadPoseAsMatrix();

    //assume intrinsic eulers and x-y-z order
    void LoadPoseAsEulerAngles();
    ci::Matrix44f MatrixFromIntrinsicEulers(float xRotation, float yRotation, float zRotation, const std::string &order) const;
    ci::Vec3f GetZYXEulersFromQuaternion(const ci::Quatf &quaternion) const;
    ci::Vec3f GetXZYEulersFromQuaternion(const ci::Quatf &quaternion) const;
    ci::Matrix44f RemoveOutOfPlaneRotation(const ci::Matrix44f &pose) const; 

    virtual void SetOffsetsToNull() override;
    
    float x_rotation_offset_;
    float y_rotation_offset_;
    float z_rotation_offset_;
    float x_translation_offset_;
    float y_translation_offset_;
    float z_translation_offset_;

    float entire_x_rotation_offset_;
    float entire_y_rotation_offset_;
    float entire_z_rotation_offset_;

    ci::Matrix44f current_user_supplied_offset_;

    ci::Quatf rotation_;
    ci::Vec3f translation_;

    std::vector<float> wrist_offsets_;

    std::size_t num_wrist_joints_; /**< Number of joints in the wrist of the instrument. */
    
    std::ifstream ifs_; /**< The file to read the DH and SE3 values from. */
    std::ofstream ofs_; /**< The file to write modified DH and SE3 values to. */
    std::string ofs_file_; /** The file name to write to. Allows delayed opening. */

    ci::Matrix44f shaft_pose_; /**< Maintain a cache of shaft pose value so that model can be refreshed without reloading. */
    std::vector<double> wrist_dh_params_; /**< Maintain a  */

  };


  /**
  * @class QuaternionPoseGrabber
  * @brief Load pose as a Quaternion + translation
  *
  */

  class QuaternionPoseGrabber : public SE3DaVinciPoseGrabber {

  public:

    /**
    * Load a pose grabber from a config file. This file contains the model coordinate file (if applicable) and pose file.
    */
    QuaternionPoseGrabber(const ConfigReader &reader, const std::string &output_dir);

    /**
    * Load the next SE3 pose transform from the file. This changes the class' internal representation of its pose when it renders the model.
    * @param[in] no_reload Just refresh the internal representation of pose without reading anything from the pose file. This is useful if the pose can be manually modified within the UI to account for a rigid offset.
    * @return The success of the load (false if for instance there are no poses left to read from the file).
    */
    virtual bool LoadPose(const bool no_reload);


  };

}