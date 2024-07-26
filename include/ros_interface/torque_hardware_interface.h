#ifndef TORQUE_HARDWARE_INTERFACE_H
#define TORQUE_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <motor_protocols/MG6012I36.h>
#include <can_msgs/Frame.h>

class TorqueHardwareInterface : public hardware_interface::RobotHW
{
	public:
        TorqueHardwareInterface(ros::NodeHandle& nh);
        ~TorqueHardwareInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        
    protected:
        std::string node_name_;
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;

        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        
        std::vector<std::string> joint_names_;
        int num_joints_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;
        std::vector<double> joint_position_command_;
        
        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        double loop_frequency_;
        std::string hardware_type_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

        MG6012I36 mg6012_protocols_;
        can_msgs::Frame joint_command_frame_;
        ros::Publisher can_frame_publisher_;
        ros::Subscriber can_frame_subscriber_;
        void CANCallback_(const can_msgs::Frame::ConstPtr& msg);
};

#endif
