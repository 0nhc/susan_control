#include <ros_interface/susan_hardware_interface.h>
#include <math.h>
SusanHardwareInterface::SusanHardwareInterface(ros::NodeHandle& nh) : nh_(nh)
{
  if(!nh_.getParam(ros::this_node::getName() + "/loop_frequency", loop_frequency_))
  {
    ROS_WARN("Could not find parameter '%s/loop_frequency', set to 100.0 Hz by default", ros::this_node::getName());
    loop_frequency_ = 100.0;
  }
  if(!nh_.getParam(ros::this_node::getName() + "/hardware_type", hardware_type_))
  {
    ROS_WARN("Could not find parameter '%s/hardware_type', set to fake by default", ros::this_node::getName());
    hardware_type_ = "fake";
  }
  if(!nh_.getParam(ros::this_node::getName() + "/feedback_type", feedback_type_))
  {
    ROS_WARN("Could not find parameter '%s/feedback_type', set to fake by default", ros::this_node::getName());
    feedback_type_ = "fake";
  }
  if(!nh_.getParam("susan_arm_group_controller/joints", joint_names_))
  {
    ROS_ERROR("Could not find parameter 'susan_arm_group_controller/joints'.");
    abort();
  }

  num_joints_ = joint_names_.size();
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);
  joint_position_command_.resize(num_joints_, 0.0);

  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
  ros::Duration update_freq = ros::Duration(1.0/loop_frequency_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &SusanHardwareInterface::update, this);
  if(feedback_type_ == "real")
  {
    can_frame_subscriber_ = nh_.subscribe("received_messages", 1, &SusanHardwareInterface::CANCallback_, this);
  }
  can_frame_publisher_ = nh_.advertise<can_msgs::Frame>("sent_messages", 1);
  usleep(1000000); // wait 1 s for initializing ROS publishers and subscribers
  init();
}

SusanHardwareInterface::~SusanHardwareInterface()
{
}

void SusanHardwareInterface::init()
{
  for(int i=0; i<num_joints_; i++)
  {
    // Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Create position joint interface
    hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
    position_joint_interface_.registerHandle(jointPositionHandle);

    // Create Joint Limit interface   
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::getJointLimits(joint_names_[i], nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle(jointPositionHandle, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle);
  }

  // Register all joints interfaces    
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&positionJointSaturationInterface);

  if(hardware_type_ == "real")
  {
    // Enable DM Motors
    for(int i=2; i<7; i++)
    {
      joint_command_frame_ = dm4340_protocols_.encodeEnableCommand(i+1);
      joint_command_frame_.header.stamp = ros::Time::now();
      joint_command_frame_.header.frame_id = "DM4340";
      can_frame_publisher_.publish(joint_command_frame_);
      usleep(1000);
    }
  }

}

void SusanHardwareInterface::update(const ros::TimerEvent& e)
{
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read();
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);
}

void SusanHardwareInterface::read()
{
  // for(int i=0; i<num_joints_; i++)
  // {
  //   ROS_INFO("Joint Position[%i]: %.2f", i, joint_position_[i]);
  // }

}

void SusanHardwareInterface::write(ros::Duration elapsed_time)
{
  if(hardware_type_ == "fake")
  {
    for(int i=0; i<7; i++)
    {
      joint_position_[i] = joint_position_command_[i];
    }
  }
  else if(hardware_type_ == "real")
  {
    positionJointSaturationInterface.enforceLimits(elapsed_time);

    joint_command_frame_ = mg6012_protocols_.encodePositionCommand(1, joint_position_command_[0]);
    joint_command_frame_.header.stamp = ros::Time::now();
    joint_command_frame_.header.frame_id = "MG6012";
    can_frame_publisher_.publish(joint_command_frame_);
    usleep(1000);

    joint_command_frame_ = mg6012_protocols_.encodePositionCommand(2, joint_position_command_[1]);
    joint_command_frame_.header.stamp = ros::Time::now();
    joint_command_frame_.header.frame_id = "MG6012";
    can_frame_publisher_.publish(joint_command_frame_);
    usleep(1000);

    joint_command_frame_ = dm4340_protocols_.encodePositionCommand(3, joint_position_command_[2], M_PI/2.0);
    joint_command_frame_.header.stamp = ros::Time::now();
    joint_command_frame_.header.frame_id = "DM4340";
    can_frame_publisher_.publish(joint_command_frame_);
    usleep(1000);

    joint_command_frame_ = dm4340_protocols_.encodePositionCommand(4, joint_position_command_[3], M_PI/2.0);
    joint_command_frame_.header.stamp = ros::Time::now();
    joint_command_frame_.header.frame_id = "DM4340";
    can_frame_publisher_.publish(joint_command_frame_);
    usleep(1000);

    joint_command_frame_ = dm4340_protocols_.encodePositionCommand(5, joint_position_command_[4], M_PI/2.0);
    joint_command_frame_.header.stamp = ros::Time::now();
    joint_command_frame_.header.frame_id = "DM4340";
    can_frame_publisher_.publish(joint_command_frame_);
    usleep(1000);

    joint_command_frame_ = dm4340_protocols_.encodePositionCommand(6, joint_position_command_[5], M_PI/2.0);
    joint_command_frame_.header.stamp = ros::Time::now();
    joint_command_frame_.header.frame_id = "DM4340";
    can_frame_publisher_.publish(joint_command_frame_);
    usleep(1000);

    joint_command_frame_ = dm4340_protocols_.encodePositionCommand(7, joint_position_command_[6], M_PI/2.0);
    joint_command_frame_.header.stamp = ros::Time::now();
    joint_command_frame_.header.frame_id = "DM4340";
    can_frame_publisher_.publish(joint_command_frame_);
    usleep(1000);

    if(feedback_type_ == "fake")
    {
      for(int i=0; i<num_joints_; i++)
      {
        joint_position_[i] = joint_position_command_[i];
      }
    }
  }

}

void SusanHardwareInterface::CANCallback_(const can_msgs::Frame::ConstPtr& msg)
{
  if(hardware_type_ == "real")
  {
    if(msg->id == 0x141 && msg->data[0] == 0xA4)
    {
      joint_position_[0] = mg6012_protocols_.decodePositionFrame(*msg);
    }
    else if(msg->id == 0x142 && msg->data[0] == 0xA4)
    {
      joint_position_[1] = mg6012_protocols_.decodePositionFrame(*msg);
    }
    else if(msg->id == 0x000 && (msg->data[0]&0x0F) == 0x03)
    {
      joint_position_[2] = dm4340_protocols_.decodePositionFrame(*msg);
    }
    else if(msg->id == 0x000 && (msg->data[0]&0x0F) == 0x04)
    {
      joint_position_[3] = dm4340_protocols_.decodePositionFrame(*msg);
    }
    else if(msg->id == 0x000 && (msg->data[0]&0x0F) == 0x05)
    {
      joint_position_[4] = dm4340_protocols_.decodePositionFrame(*msg);
    }
    else if(msg->id == 0x000 && (msg->data[0]&0x0F) == 0x06)
    {
      joint_position_[5] = dm4340_protocols_.decodePositionFrame(*msg);
    }
    else if(msg->id == 0x000 && (msg->data[0]&0x0F) == 0x07)
    {
      joint_position_[6] = dm4340_protocols_.decodePositionFrame(*msg);
    }
  }

}
