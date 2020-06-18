#ifndef _MOTOR_PLUGIN_H_
#define _MOTOR_PLUGIN_H_

#include <map>
#include <thread>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

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

    protected:

      virtual void UpdateChild();
      virtual void FiniChild();

    private:

      GazeboRosPtr gazebo_ros_;
      event::ConnectionPtr update_connection_;
      physics::ModelPtr parent;
      physics::JointPtr joint_;
      physics::LinkPtr link_;
      ros::Publisher encoder_publisher_;
      ros::Publisher velocity_publisher_;
      ros::Publisher joint_state_publisher_;
      ros::Subscriber cmd_vel_subscriber_;
      sensor_msgs::JointState joint_state_;

      // Topic params
      std::string command_topic_;
      std::string encoder_topic_;
      std::string velocity_topic_;
      bool publish_velocity_;
      bool publish_encoder_;
      bool publish_motor_joint_state_;

      // ODE
      double input_;
      double update_rate_;
      double ode_joint_motor_fmax_;
      double ode_joint_fudge_factor_;

      // Encoder model
      int encoder_counter_;
      int encoder_pulses_per_revolution_;
      double encoder_to_shaft_ratio_;

      // Callback Queue
      ros::CallbackQueue queue_;
      std::thread callback_queue_thread_;
      boost::mutex lock_;
      void QueueThread();

      // Helper variables
      double update_period_;
      common::Time last_update_time_;

      void publishRotorVelocity(double m_vel);
      void publishEncoderCount(long ctr);
      void cmdVelCallback(const std_msgs::Float32::ConstPtr& cmd_msg);
      void publishWheelJointState();
      void publishEncoderCount(double m_vel, double dT);
  };

}

#endif
