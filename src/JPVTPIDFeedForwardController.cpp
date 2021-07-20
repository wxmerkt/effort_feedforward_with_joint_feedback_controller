// Copyright 2021 Wolfgang Merkt

#include <mutex>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/position_velocity_torque_gains_command_interface.h>
#include <ipab_controller_msgs/EffortFeedforwardWithJointFeedbackTrajectory.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>

// #include <joint_limits_interface/joint_limits.h>
// #include <joint_limits_interface/joint_limits_urdf.h>
// #include <joint_limits_interface/joint_limits_rosparam.h>

inline double clamp(double x, double lower, double upper)
{
    return std::max(lower, std::min(upper, x));
}

namespace effort_feedforward_with_joint_feedback_controller
{
class JPVTPIDFeedForwardController
    : public controller_interface::Controller<hardware_interface::PositionVelocityTorqueGainsJointInterface>
{
public:
    JPVTPIDFeedForwardController() {}
    ~JPVTPIDFeedForwardController() { sub_command_.shutdown(); }
    bool init(hardware_interface::PositionVelocityTorqueGainsJointInterface* hw, ros::NodeHandle& n)
    {
        // List of controlled joints
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR_STREAM("Failed to get list of joints (namespace: " << n.getNamespace() << ").");
            return false;
        }
        n_joints_ = joint_names_.size();

        // Get URDF
        urdf::Model urdf;
        if (!urdf.initParamWithNodeHandle("robot_description", n))
        {
            ROS_ERROR("Failed to parse URDF file");
            return false;
        }

        if (n_joints_ == 0)
        {
            ROS_ERROR_STREAM("List of joint names is empty.");
            return false;
        }
        for (std::size_t i = 0; i < n_joints_; i++)
        {
            const auto& joint_name = joint_names_[i];

            try
            {
                joints_.push_back(hw->getHandle(joint_name));
            }
            catch (const hardware_interface::HardwareInterfaceException& e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
            if (!joint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf);
        }

        // default_positions
        if (!n.getParam("default_positions", default_positions_) || default_positions_.size() != n_joints_)
        {
            ROS_ERROR_STREAM("default_positions incorrect.");
            return false;
        }

        // default_position_gains
        if (!n.getParam("default_position_gains", default_position_gains_) || default_position_gains_.size() != n_joints_)
        {
            ROS_ERROR_STREAM("default_position_gains incorrect.");
            return false;
        }

        // default_velocity_gains
        if (!n.getParam("default_velocity_gains", default_velocity_gains_) || default_velocity_gains_.size() != n_joints_)
        {
            ROS_ERROR_STREAM("default_velocity_gains incorrect.");
            return false;
        }

        // Initialise real-time thread-safe buffers
        default_velocities_.assign(n_joints_, 0.0);
        default_efforts_.assign(n_joints_, 0.0);
        desired_positions_buffer_.writeFromNonRT(default_positions_);
        desired_velocities_buffer_.writeFromNonRT(default_velocities_);
        desired_efforts_buffer_.writeFromNonRT(default_efforts_);
        position_gains_buffer_.writeFromNonRT(default_position_gains_);
        velocity_gains_buffer_.writeFromNonRT(default_velocity_gains_);

        // Subscribe to command topic
        sub_command_ = n.subscribe<ipab_controller_msgs::EffortFeedforwardWithJointFeedbackTrajectory>("command", 1, &JPVTPIDFeedForwardController::commandCB, this);

        // Joint limits
        // joint_limits_interface::JointLimits limits;

        return true;
    }

    void starting(const ros::Time& /*time*/)
    {
        // Starting from the current position
        std::vector<double> q_current(n_joints_);
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            q_current[i] = joints_[i].getPosition();
        }
        desired_positions_buffer_.readFromRT()->assign(q_current.begin(), q_current.end());

        // Reset velocities and efforts
        desired_velocities_buffer_.readFromRT()->assign(default_velocities_.begin(), default_velocities_.end());
        desired_efforts_buffer_.readFromRT()->assign(default_efforts_.begin(), default_efforts_.end());
        // position_gains_buffer_.readFromRT()->assign(default_position_gains_.begin(), default_position_gains_.end());
        // velocity_gains_buffer_.readFromRT()->assign(default_velocity_gains_.begin(), default_velocity_gains_.end());
    }

    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
    {
        advanceTrajectory();

        std::vector<double>& desired_positions = *desired_positions_buffer_.readFromRT();
        std::vector<double>& desired_velocities = *desired_velocities_buffer_.readFromRT();
        std::vector<double>& desired_efforts = *desired_efforts_buffer_.readFromRT();
        std::vector<double>& position_gains = *position_gains_buffer_.readFromRT();
        std::vector<double>& velocity_gains = *velocity_gains_buffer_.readFromRT();

        // Assign
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            joints_[i].setCommand(desired_positions[i],
                                  desired_velocities[i],
                                  desired_efforts[i],
                                  position_gains[i],
                                  velocity_gains[i]);
        }
    }

protected:
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::PositionVelocityTorqueGainsJointHandle> joints_;
    std::size_t n_joints_;

    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

    // Default values
    std::vector<double> default_positions_;
    std::vector<double> default_velocities_;
    std::vector<double> default_efforts_;
    std::vector<double> default_position_gains_;
    std::vector<double> default_velocity_gains_;

    // Real-time buffers
    realtime_tools::RealtimeBuffer<std::vector<double>> desired_positions_buffer_;
    realtime_tools::RealtimeBuffer<std::vector<double>> desired_velocities_buffer_;
    realtime_tools::RealtimeBuffer<std::vector<double>> desired_efforts_buffer_;
    realtime_tools::RealtimeBuffer<std::vector<double>> position_gains_buffer_;
    realtime_tools::RealtimeBuffer<std::vector<double>> velocity_gains_buffer_;

    std::mutex mutex_;
    int index_in_trajectory_ = -1;
    double trajectory_dt_;
    ipab_controller_msgs::EffortFeedforwardWithJointFeedbackTrajectory trajectory_;
    void advanceTrajectory()
    {
        if (index_in_trajectory_ == -1 || index_in_trajectory_ == static_cast<int>(trajectory_.trajectory.size()) - 1)
        {
            ROS_WARN_STREAM_THROTTLE(5, "No trajectory received || End of trajectory");
            return;
        }

        mutex_.lock();

        // Positions
        if (trajectory_.trajectory[index_in_trajectory_].positions.size() == n_joints_)
        {
            desired_positions_buffer_.writeFromNonRT(trajectory_.trajectory[index_in_trajectory_].positions);
        }
        else if (trajectory_.trajectory[index_in_trajectory_].positions.size() == 0)
        {
            ROS_DEBUG_STREAM("No desired positions set.");
        }
        else
        {
            ROS_ERROR_STREAM("Number of desired positions wrong: got " << trajectory_.trajectory[index_in_trajectory_].positions.size() << " expected " << n_joints_ << " or 0.");
        }

        // Velocities
        if (trajectory_.trajectory[index_in_trajectory_].velocities.size() == n_joints_)
        {
            desired_velocities_buffer_.writeFromNonRT(trajectory_.trajectory[index_in_trajectory_].velocities);
        }
        else if (trajectory_.trajectory[index_in_trajectory_].velocities.size() == 0)
        {
            ROS_DEBUG_STREAM("No desired velocities set.");
        }
        else
        {
            ROS_ERROR_STREAM("Number of desired velocities wrong: got " << trajectory_.trajectory[index_in_trajectory_].velocities.size() << " expected " << n_joints_ << " or 0.");
        }

        // Efforts
        if (trajectory_.trajectory[index_in_trajectory_].efforts.size() == n_joints_)
        {
            desired_efforts_buffer_.writeFromNonRT(trajectory_.trajectory[index_in_trajectory_].efforts);
        }
        else if (trajectory_.trajectory[index_in_trajectory_].efforts.size() == 0)
        {
            ROS_DEBUG_STREAM("No desired efforts set.");
        }
        else
        {
            ROS_ERROR_STREAM("Number of desired efforts wrong: got " << trajectory_.trajectory[index_in_trajectory_].efforts.size() << " expected " << n_joints_ << " or 0.");
        }

        // Position Gains
        if (trajectory_.trajectory[index_in_trajectory_].position_gains.size() == n_joints_)
        {
            position_gains_buffer_.writeFromNonRT(trajectory_.trajectory[index_in_trajectory_].position_gains);
        }
        else if (trajectory_.trajectory[index_in_trajectory_].position_gains.size() == 0)
        {
            ROS_DEBUG_STREAM("No desired position_gains set.");
        }
        else
        {
            ROS_ERROR_STREAM("Number of position gains wrong: got " << trajectory_.trajectory[index_in_trajectory_].position_gains.size() << " expected " << n_joints_ << " or 0.");
        }

        // Velocity Gains
        if (trajectory_.trajectory[index_in_trajectory_].velocity_gains.size() == n_joints_)
        {
            velocity_gains_buffer_.writeFromNonRT(trajectory_.trajectory[index_in_trajectory_].velocity_gains);
        }
        else if (trajectory_.trajectory[index_in_trajectory_].velocity_gains.size() == 0)
        {
            ROS_DEBUG_STREAM("No desired velocity_gains set.");
        }
        else
        {
            ROS_ERROR_STREAM("Number of velocity gains wrong: got " << trajectory_.trajectory[index_in_trajectory_].velocity_gains.size() << " expected " << n_joints_ << " or 0.");
        }

        // Increment
        index_in_trajectory_++;

        mutex_.unlock();
    }

    // Subscriber for new commands (real-time safe)
    ros::Subscriber sub_command_;
    void commandCB(const ipab_controller_msgs::EffortFeedforwardWithJointFeedbackTrajectoryConstPtr& msg)
    {
        ROS_INFO_STREAM(">>> New trajectory received");
        mutex_.lock();
        trajectory_dt_ = msg->dt;
        index_in_trajectory_ = 0;
        trajectory_ = *msg;
        std::cout << "dt " << msg->dt << " - length: " << trajectory_.trajectory.size() << std::endl;
        mutex_.unlock();
    }
};
}  // namespace effort_feedforward_with_joint_feedback_controller

PLUGINLIB_EXPORT_CLASS(effort_feedforward_with_joint_feedback_controller::JPVTPIDFeedForwardController, controller_interface::ControllerBase)
