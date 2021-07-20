// Copyright 2019 Wolfgang Merkt

#include <string>
#include <vector>

#include <angles/angles.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ipab_controller_msgs/EffortFeedforwardWithJointFeedback.h>
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
class EffortFeedforwardWithJointFeedbackController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    EffortFeedforwardWithJointFeedbackController() {}
    ~EffortFeedforwardWithJointFeedbackController() { sub_command_.shutdown(); }
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n)
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

        // Offset in command buffer
        if (!n.getParam("command_offset", command_offset_))
        {
            command_offset_ = 0;
        }

        if (command_offset_ < 0)
        {
            ROS_ERROR_STREAM("Negative command offset (" << command_offset_ << ")...");
            return false;
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

        // max_efforts
        if (!n.getParam("max_efforts", max_efforts_) || max_efforts_.size() != n_joints_)
        {
            ROS_ERROR_STREAM("max_efforts incorrect: " << max_efforts_.size());
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
        sub_command_ = n.subscribe<ipab_controller_msgs::EffortFeedforwardWithJointFeedback>("command", 1, &EffortFeedforwardWithJointFeedbackController::commandCB, this);

        // Start realtime state publisher
        feedback_control_ratio_pub_.reset(
            new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(n, "feedback_control_ratio", 1));

        if (feedback_control_ratio_pub_ && feedback_control_ratio_pub_->trylock())
        {
            feedback_control_ratio_pub_->msg_.data.assign(n_joints_, 0.0);
        }

        // Joint limits
        // joint_limits_interface::JointLimits limits;

        return true;
    }

    void starting(const ros::Time& /*time*/)
    {
        // TODO: START FROM CURRENT POSITION (!!!)
        std::vector<double> q_current(n_joints_);
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            q_current[i] = joints_[i].getPosition();
        }
        desired_positions_buffer_.readFromRT()->assign(q_current.begin(), q_current.end());
    }

    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
    {
        std::vector<double>& desired_positions = *desired_positions_buffer_.readFromRT();
        std::vector<double>& desired_velocities = *desired_velocities_buffer_.readFromRT();
        std::vector<double>& desired_efforts = *desired_efforts_buffer_.readFromRT();
        std::vector<double>& position_gains = *position_gains_buffer_.readFromRT();
        std::vector<double>& velocity_gains = *velocity_gains_buffer_.readFromRT();

        std::vector<double> effort_ratios(n_joints_);
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            // Compute control
            double q_measured = joints_[i].getPosition();
            double qdot_measured = joints_[i].getVelocity();

            double position_error, velocity_error;

            // Compute position error
            if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
            {
                angles::shortest_angular_distance_with_large_limits(
                    q_measured,
                    desired_positions[i],
                    joint_urdfs_[i]->limits->lower,
                    joint_urdfs_[i]->limits->upper,
                    position_error);
            }
            else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
            {
                position_error = angles::shortest_angular_distance(q_measured, desired_positions[i]);
            }
            else  //prismatic
            {
                position_error = desired_positions[i] - q_measured;
            }

            velocity_error = (desired_velocities[i] - qdot_measured);

            double feedback_term = position_gains[i] * position_error + velocity_gains[i] * velocity_error;
            double effort = desired_efforts[i] + feedback_term;
            double clamped_effort = clamp(effort, -joint_urdfs_[i]->limits->effort, joint_urdfs_[i]->limits->effort);

            joints_[i].setCommand(clamped_effort);

            // compute ratio and send
            effort_ratios[i] = std::abs(feedback_term) / std::abs(effort);
        }

        if (feedback_control_ratio_pub_ && feedback_control_ratio_pub_->trylock())
        {
            feedback_control_ratio_pub_->msg_.data.resize(n_joints_);
            feedback_control_ratio_pub_->msg_.data = effort_ratios;
            feedback_control_ratio_pub_->unlockAndPublish();
        }
    }

protected:
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    std::size_t n_joints_;
    int command_offset_ = 0;

    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

    // Default values
    std::vector<double> default_positions_;
    std::vector<double> default_velocities_;
    std::vector<double> default_efforts_;
    std::vector<double> default_position_gains_;
    std::vector<double> default_velocity_gains_;
    std::vector<double> max_efforts_;

    // Real-time buffers
    realtime_tools::RealtimeBuffer<std::vector<double>> desired_positions_buffer_;
    realtime_tools::RealtimeBuffer<std::vector<double>> desired_velocities_buffer_;
    realtime_tools::RealtimeBuffer<std::vector<double>> desired_efforts_buffer_;
    realtime_tools::RealtimeBuffer<std::vector<double>> position_gains_buffer_;
    realtime_tools::RealtimeBuffer<std::vector<double>> velocity_gains_buffer_;

    // Publisher for debugging (real-time safe)
    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> feedback_control_ratio_pub_;

    // Subscriber for new commands (real-time safe)
    ros::Subscriber sub_command_;
    void commandCB(const ipab_controller_msgs::EffortFeedforwardWithJointFeedbackConstPtr& msg)
    {
        // Positions
        if (msg->positions.size() == n_joints_)
        {
            desired_positions_buffer_.writeFromNonRT(msg->positions);
        }
        else if (msg->positions.size() > n_joints_ && msg->positions.size() <= n_joints_ + command_offset_)
        {
            std::vector<double> positions(msg->positions.begin() + command_offset_, msg->positions.begin() + command_offset_ + n_joints_);
            desired_positions_buffer_.writeFromNonRT(positions);
        }
        else if (msg->positions.size() == 0)
        {
            ROS_DEBUG_STREAM("No desired positions set.");
        }
        else
        {
            ROS_ERROR_STREAM("Number of desired positions wrong: got " << msg->positions.size() << " expected " << n_joints_ << " or 0.");
        }

        // Velocities
        if (msg->velocities.size() == n_joints_)
        {
            desired_velocities_buffer_.writeFromNonRT(msg->velocities);
        }
        else if (msg->velocities.size() > n_joints_ && msg->velocities.size() <= n_joints_ + command_offset_)
        {
            std::vector<double> velocities(msg->velocities.begin() + command_offset_, msg->velocities.begin() + command_offset_ + n_joints_);
            desired_velocities_buffer_.writeFromNonRT(velocities);
        }
        else if (msg->velocities.size() == 0)
        {
            ROS_DEBUG_STREAM("No desired velocities set.");
        }
        else
        {
            ROS_ERROR_STREAM("Number of desired velocities wrong: got " << msg->velocities.size() << " expected " << n_joints_ << " or 0.");
        }

        // Efforts
        if (msg->efforts.size() == n_joints_)
        {
            desired_efforts_buffer_.writeFromNonRT(msg->efforts);
        }
        else if (msg->efforts.size() > n_joints_ && msg->efforts.size() <= n_joints_ + command_offset_)
        {
            std::vector<double> efforts(msg->efforts.begin() + command_offset_, msg->efforts.begin() + command_offset_ + n_joints_);
            desired_efforts_buffer_.writeFromNonRT(efforts);
        }
        else if (msg->efforts.size() == 0)
        {
            ROS_DEBUG_STREAM("No desired efforts set.");
        }
        else
        {
            ROS_ERROR_STREAM("Number of desired efforts wrong: got " << msg->efforts.size() << " expected " << n_joints_ << " or 0.");
        }

        // Position Gains
        if (msg->position_gains.size() == n_joints_)
        {
            position_gains_buffer_.writeFromNonRT(msg->position_gains);
        }
        else if (msg->position_gains.size() > n_joints_ && msg->position_gains.size() <= n_joints_ + command_offset_)
        {
            std::vector<double> position_gains(msg->position_gains.begin() + command_offset_, msg->position_gains.begin() + command_offset_ + n_joints_);
            position_gains_buffer_.writeFromNonRT(position_gains);
        }
        else if (msg->position_gains.size() == 0)
        {
            ROS_DEBUG_STREAM("No desired position_gains set.");
        }
        else
        {
            ROS_ERROR_STREAM("Number of position gains wrong: got " << msg->position_gains.size() << " expected " << n_joints_ << " or 0.");
        }

        // Velocity Gains
        if (msg->velocity_gains.size() == n_joints_)
        {
            velocity_gains_buffer_.writeFromNonRT(msg->velocity_gains);
        }
        else if (msg->velocity_gains.size() > n_joints_ && msg->velocity_gains.size() <= n_joints_ + command_offset_)
        {
            std::vector<double> velocity_gains(msg->velocity_gains.begin() + command_offset_, msg->velocity_gains.begin() + command_offset_ + n_joints_);
            velocity_gains_buffer_.writeFromNonRT(velocity_gains);
        }
        else if (msg->velocity_gains.size() == 0)
        {
            ROS_DEBUG_STREAM("No desired velocity_gains set.");
        }
        else
        {
            ROS_ERROR_STREAM("Number of velocity gains wrong: got " << msg->velocity_gains.size() << " expected " << n_joints_ << " or 0.");
        }
    }
};
}  // namespace effort_feedforward_with_joint_feedback_controller

PLUGINLIB_EXPORT_CLASS(effort_feedforward_with_joint_feedback_controller::EffortFeedforwardWithJointFeedbackController, controller_interface::ControllerBase)
