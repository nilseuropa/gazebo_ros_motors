#include <algorithm>
#include <assert.h>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>

#include "gazebo_ros_joint_motor/gazebo_ros_joint_motor.h"

namespace gazebo {

// Constructor
GazeboRosMotor::GazeboRosMotor() {
}

// Destructor
GazeboRosMotor::~GazeboRosMotor() {
	FiniChild();
}

// Load the controller
void GazeboRosMotor::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    this->parent = _parent;

    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "MotorPlugin" ) );
    gazebo_ros_->isInitialized();

		// topics
    gazebo_ros_->getParameter<std::string> ( command_topic_,  "command_topic",  "/motor/command" );
		gazebo_ros_->getParameter<std::string> ( velocity_topic_, "velocity_topic", "/motor/velocity" );
    gazebo_ros_->getParameter<std::string> ( encoder_topic_,  "encoder_topic",  "/motor/encoder" );

		// ode_joint_motor_fmax_ = maximum force the ODE joint-motor may apply during a time step
		gazebo_ros_->getParameter<double> ( ode_joint_motor_fmax_, "ode_joint_motor_fmax", 50.0 );
		gazebo_ros_->getParameter<double> ( ode_joint_fudge_factor_, "ode_joint_fudge_factor", 1.0 );
		gazebo_ros_->getParameter<double> ( update_rate_, "update_rate", 100.0 );

		// encoder parameters
		gazebo_ros_->getParameterBoolean  ( publish_velocity_, "publish_velocity", true );
		gazebo_ros_->getParameterBoolean  ( publish_encoder_, "publish_encoder", false );
		gazebo_ros_->getParameter<int> 		( encoder_pulses_per_revolution_, "encoder_ppr", 4096 );
		gazebo_ros_->getParameter<double> ( encoder_to_shaft_ratio_, "encoder_to_shaft_ratio", 1.0 );

		// motor joint
    joint_ = gazebo_ros_->getJoint ( parent, "motor_shaft_joint", "shaft_joint" );
		joint_->SetParam( "fmax", 0, this->ode_joint_motor_fmax_ );
		joint_->SetParam( "fudge_factor", 0, this->ode_joint_fudge_factor_);
		joint_->SetProvideFeedback(true);

		// joint state publisher
		gazebo_ros_->getParameterBoolean  ( publish_motor_joint_state_, "publish_motor_joint_state", false );
    if (this->publish_motor_joint_state_) {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_state", 1000);
        ROS_INFO_NAMED("motor_plugin", "%s: Advertise joint_state", gazebo_ros_->info());
    }

		// command subscription
		if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_; else this->update_period_ = 0.0;
		last_update_time_ = parent->GetWorld()->GetSimTime();
    ROS_INFO_NAMED("motor_plugin", "%s: Trying to subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32> (
        command_topic_,
				1,
        boost::bind(&GazeboRosMotor::cmdVelCallback, this, _1),
        ros::VoidPtr(),
        &queue_
    );
    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("motor_plugin", "%s: Subscribed to %s", gazebo_ros_->info(), command_topic_.c_str());

		if (this->publish_velocity_){
	    velocity_publisher_ = gazebo_ros_->node()->advertise<std_msgs::Float32>(velocity_topic_, 1);
	    ROS_INFO_NAMED("motor_plugin", "%s: Advertising shaft velocity on %s ", gazebo_ros_->info(), velocity_topic_.c_str());
		}

		if (this->publish_encoder_){
			encoder_publisher_ = gazebo_ros_->node()->advertise<std_msgs::Int32>(encoder_topic_, 1);
	    ROS_INFO_NAMED("motor_plugin", "%s: Advertising encoder counts on %s ", gazebo_ros_->info(), encoder_topic_.c_str());
		}

    // start custom queue
    this->callback_queue_thread_ = std::thread ( std::bind ( &GazeboRosMotor::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosMotor::UpdateChild, this ) );

		input_ = 0;
		encoder_counter_ = 0;
}

void GazeboRosMotor::Reset() {
  last_update_time_ = parent->GetWorld()->GetSimTime();
  joint_->SetParam ( "fmax", 0, ode_joint_motor_fmax_ );
	joint_->SetVelocity(0,0);
  input_ = 0;
	encoder_counter_ = 0;
}

void GazeboRosMotor::publishWheelJointState() {
	if (this->publish_motor_joint_state_){
    ros::Time current_time = ros::Time::now();
    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( 1 );
    joint_state_.position.resize ( 1 );
    physics::JointPtr joint = joint_;
    double position = joint->GetAngle(0).Radian();
    joint_state_.name[0] = joint->GetName();
    joint_state_.position[0] = position;
    joint_state_publisher_.publish ( joint_state_ );
	}
}

// Velocity publisher
void GazeboRosMotor::publishRotorVelocity(double m_vel){
  std_msgs::Float32 vel_msg;
  vel_msg.data = m_vel; // (rad/sec)
	if (this->publish_velocity_) velocity_publisher_.publish(vel_msg);
}

// Simple incremental encoder emulation
void GazeboRosMotor::publishEncoderCount(double m_vel, double dT){
	std_msgs::Int32 counter_msg;
	double rev_in_rad = m_vel * dT;
	encoder_counter_ += round( ((rev_in_rad)/2*M_PI) * encoder_pulses_per_revolution_ );
	counter_msg.data = encoder_counter_;
	if (this->publish_encoder_) encoder_publisher_.publish(counter_msg);
}

// Plugin update function
void GazeboRosMotor::UpdateChild() {
    common::Time current_time = parent->GetWorld()->GetSimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
		double current_speed = joint_->GetVelocity( 0u )*encoder_to_shaft_ratio_;
		joint_->SetParam("fmax", 0, ode_joint_motor_fmax_);
		joint_->SetParam("vel",  0, input_);
    if ( seconds_since_last_update > update_period_ ) {
				publishWheelJointState();
				publishRotorVelocity( current_speed );
				publishEncoderCount( current_speed , seconds_since_last_update );
				last_update_time_+= common::Time ( update_period_ );
    }
}

// Finalize the controller
void GazeboRosMotor::FiniChild() {
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

// Callback from custom que
void GazeboRosMotor::cmdVelCallback ( const std_msgs::Float32::ConstPtr& cmd_msg ) {
    input_ = cmd_msg->data;
}

void GazeboRosMotor::QueueThread() {
    static const double timeout = 0.01;
    while ( gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosMotor )
// eof_ns
}
