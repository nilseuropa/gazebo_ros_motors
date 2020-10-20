#ifndef _DC_MOTOR_PLUGIN_H_
#define _DC_MOTOR_PLUGIN_H_

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// configuration
#include <dynamic_reconfigure/server.h>
#include <gazebo_ros_motors/motorModelConfig.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosMotor : public ModelPlugin {

    public:

      GazeboRosMotor();
      ~GazeboRosMotor();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Reset();
	  bool ValidateParameters();

    protected:

      virtual void UpdateChild();
      virtual void FiniChild();

    private:

      GazeboRosPtr gazebo_ros_;
      std::string plugin_name_;
      event::ConnectionPtr update_connection_;
      physics::ModelPtr parent;
      physics::JointPtr joint_;
      physics::LinkPtr link_;
      ros::Publisher encoder_publisher_;
      ros::Publisher velocity_publisher_;
      ros::Publisher current_publisher_;
      ros::Publisher joint_state_publisher_;
      ros::Subscriber cmd_vel_subscriber_;
      sensor_msgs::JointState joint_state_;
      geometry_msgs::WrenchStamped wrench_msg_;

      // Measurement noise
      double velocity_noise_;

      // Topic params
      std::string command_topic_;
      std::string encoder_topic_;
      std::string velocity_topic_; /// topic for the motor shaft velocity (encoder side, before gearbox)
      std::string current_topic_;
      std::string wrench_frame_;
      bool publish_velocity_;
      bool publish_current_;
      bool publish_encoder_;
      bool publish_motor_joint_state_;
      double input_;
      double update_rate_;

      // Gearbox
      double gear_ratio_; /// reduction ratio, eg 10.0 means 1/10-th output angular velocity compared to motor inner vel.

      // Motor model
      double motor_nominal_voltage_; /// the nominal voltage of the motor which corresponds to max angular velocity
      double moment_of_inertia_;
      double armature_damping_ratio_;
      double electromotive_force_constant_; // Nm/A = V/(rad/s)
      double electric_resistance_;
      double electric_inductance_;
      gazebo_ros_motors::motorModelConfig current_config_;
      // Internal state variables
      double internal_current_;
      double internal_omega_;

      // Encoder model
      int encoder_counter_;
      int encoder_pulses_per_revolution_;

      // Callback Queue
      ros::CallbackQueue queue_;
      std::thread callback_queue_thread_;
      boost::mutex lock_;
      void QueueThread();

      // Reconfiguration
      boost::shared_ptr<dynamic_reconfigure::Server<gazebo_ros_motors::motorModelConfig>> dynamic_reconfigure_server_;
      boost::recursive_mutex reconf_mutex_;
      bool notify_server_;
      void reconfigureCallBack(const gazebo_ros_motors::motorModelConfig &config, uint32_t level);
      ros::NodeHandle* node_handle_;

      // Helper variables
      double update_period_;
      common::Time last_update_time_;

      void publishRotorVelocity(double m_vel);
      void publishEncoderCount(long ctr);
      void cmdVelCallback(const std_msgs::Float32::ConstPtr& cmd_msg);
      void publishWheelJointState( double m_vel, double m_effort );
      void motorModelUpdate(double dt, double actual_omega, double current_torque);
      void publishEncoderCount(double m_vel, double dT);
      void publishMotorCurrent();
  };

}

#endif
