#include <algorithm>
#include <assert.h>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>

#include "gazebo_ros_dc_motor/gazebo_ros_dc_motor.h"

namespace gazebo {

// Constructor
GazeboRosMotor::GazeboRosMotor() {
}

// Destructor
GazeboRosMotor::~GazeboRosMotor() {
  FiniChild();
}

// Load the controller
void GazeboRosMotor::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf ) {
    this->parent = _parent;

    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "MotorPlugin" ) );
    gazebo_ros_->isInitialized();

    // global parameters
    gazebo_ros_->getParameter<std::string> ( command_topic_,  "command_topic",  "/motor/command" );
    gazebo_ros_->getParameter<double> ( update_rate_, "update_rate", 100.0 );

    // motor model parameters
    gazebo_ros_->getParameter<double> ( motor_nominal_voltage_, "motor_nominal_voltage", 24.0 ); // Datasheet 24.0V
    gazebo_ros_->getParameter<double> ( moment_of_inertia_, "moment_of_inertia", 0.001 ); // 0.001 kgm^2
    gazebo_ros_->getParameter<double> ( armature_damping_ratio_, "armature_damping_ratio", 0.0001 ); // Nm/(rad/s)
    gazebo_ros_->getParameter<double> ( electromotive_force_constant_, "electromotive_force_constant", 0.08 ); // Datasheet: 1.8Nm / 22A or 24V / 300 (rad/s)
    gazebo_ros_->getParameter<double> ( electric_resistance_, "electric_resistance", 1.0 ); // 1 Ohm
    gazebo_ros_->getParameter<double> ( electric_inductance_, "electric_inductance", 0.001 ); // 1 mH

    // gearbox parameters
    gazebo_ros_->getParameter<double> ( gear_ratio_, "gear_ratio", 1.0 ); // Reduction!

    // encoder parameters
    gazebo_ros_->getParameterBoolean  ( publish_velocity_, "publish_velocity", true );
    gazebo_ros_->getParameterBoolean  ( publish_encoder_, "publish_encoder", false );
    gazebo_ros_->getParameterBoolean  ( publish_wrench_, "publish_wrench", false );
    gazebo_ros_->getParameterBoolean  ( publish_current_, "publish_current", true );
    gazebo_ros_->getParameter<int>    ( encoder_pulses_per_revolution_, "encoder_ppr", 4096 );
    gazebo_ros_->getParameter<std::string> ( velocity_topic_, "velocity_topic", "/motor/velocity" );
    gazebo_ros_->getParameter<std::string> ( encoder_topic_,  "encoder_topic",  "/motor/encoder"  );
    gazebo_ros_->getParameter<std::string> ( wrench_topic_,   "wrench_topic",   "/motor/wrench"   );
    gazebo_ros_->getParameter<std::string> ( current_topic_,  "current_topic",  "/motor/current"  );

    // motor joint
    joint_ = gazebo_ros_->getJoint ( parent, "motor_shaft_joint", "shaft_joint" );


    // shaft link
    gazebo_ros_->getParameter<std::string> ( wrench_frame_,  "motor_wrench_frame", "wheel_link" );
    this->link_ = parent->GetLink(this->wrench_frame_);
    if (!this->link_) {
      ROS_FATAL_NAMED("motor_plugin", "link named: %s does not exist\n",this->wrench_frame_.c_str());
      return;
    }

    // joint state publisher
    gazebo_ros_->getParameterBoolean  ( publish_motor_joint_state_, "publish_motor_joint_state", false );
    if (this->publish_motor_joint_state_) {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_state", 1000);
        ROS_INFO_NAMED("motor_plugin", "%s: Advertise joint_state", gazebo_ros_->info());
    }

    // command subscriber
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_; else this->update_period_ = 0.0;
    last_update_time_ = parent->GetWorld()->SimTime();
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
    // encoder publishers
    if (this->publish_velocity_){
      velocity_publisher_ = gazebo_ros_->node()->advertise<std_msgs::Float32>(velocity_topic_, 1);
      ROS_INFO_NAMED("motor_plugin", "%s: Advertising motor shaft (before gearbox) velocity on %s ", gazebo_ros_->info(), velocity_topic_.c_str());
    }
    if (this->publish_encoder_){
      encoder_publisher_ = gazebo_ros_->node()->advertise<std_msgs::Int32>(encoder_topic_, 1);
      ROS_INFO_NAMED("motor_plugin", "%s: Advertising encoder counts on %s ", gazebo_ros_->info(), encoder_topic_.c_str());
    }
    if (this->publish_wrench_){
      wrench_publisher_ = gazebo_ros_->node()->advertise<geometry_msgs::WrenchStamped>(wrench_topic_, 1);
      ROS_INFO_NAMED("motor_plugin", "%s: Advertising joint torque and force on %s ", gazebo_ros_->info(), wrench_topic_.c_str());
    }
    if (this->publish_current_){
      current_publisher_ = gazebo_ros_->node()->advertise<std_msgs::Float32>(current_topic_, 1);
      ROS_INFO_NAMED("motor_plugin", "%s: Advertising actual motor current on %s ", gazebo_ros_->info(), current_topic_.c_str());
    }

    // start custom queue
    this->callback_queue_thread_ = std::thread ( std::bind ( &GazeboRosMotor::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosMotor::UpdateChild, this ) );

    input_ = 0;
    encoder_counter_ = 0;
    internal_current_ = 0;
    internal_omega_ = 0;
}

void GazeboRosMotor::Reset() {
  last_update_time_ = parent->GetWorld()->SimTime();
  input_ = 0;
  encoder_counter_ = 0;
  internal_current_ = 0;
  internal_omega_ = 0;
}

void GazeboRosMotor::publishWheelJointState() {
  if (this->publish_motor_joint_state_){
    ros::Time current_time = ros::Time::now();
    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( 1 );
    joint_state_.position.resize ( 1 );
    physics::JointPtr joint = joint_;
    double position = joint->Position ( 0 );
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

void GazeboRosMotor::publishMotorCurrent(){
  std_msgs::Float32 c_msg;
  c_msg.data = internal_current_; // (amps)
  if (this->publish_current_) current_publisher_.publish(c_msg);
}

void GazeboRosMotor::publishJointWrench(physics::JointWrench wrench, common::Time time){
  if (this->publish_wrench_){
    ignition::math::Vector3d torque;
    ignition::math::Vector3d force;
    force  = wrench.body2Force;
    torque = wrench.body2Torque;
    this->wrench_msg_.header.frame_id = this->wrench_frame_;
    this->wrench_msg_.header.stamp.sec = time.sec;
    this->wrench_msg_.header.stamp.nsec = time.nsec;
    this->wrench_msg_.wrench.force.x  = force.X();
    this->wrench_msg_.wrench.force.y  = force.Y();
    this->wrench_msg_.wrench.force.z  = force.Z();
    this->wrench_msg_.wrench.torque.x = torque.X();
    this->wrench_msg_.wrench.torque.y = torque.Y();
    this->wrench_msg_.wrench.torque.z = torque.Z();
    this->wrench_publisher_.publish(this->wrench_msg_);
  }
}

// Motor Model update function
void GazeboRosMotor::motorModelUpdate(double dt, double output_shaft_omega, physics::JointWrench actual_joint_wrench) {
    if (input_ > 1.0) {
        input_ = 1.0;
    } else if (input_ < -1.0) {
        input_ = -1.0;
    }
    double T = actual_joint_wrench.body1Torque.Z() / gear_ratio_; // external loading torque converted to internal side
    double V = input_ * motor_nominal_voltage_; // input voltage (command input for motor velocity)
    internal_omega_ = output_shaft_omega * gear_ratio_; // external shaft angular veloc. converted to internal side
    // DC motor exact solution for current and angular velocity (omega)
    const double& d = armature_damping_ratio_;
    const double& L = electric_inductance_;
    const double& R = electric_resistance_;
    const double& Km = electromotive_force_constant_;
    const double& J = moment_of_inertia_;
    double i0 = internal_current_;
    double o0 = internal_omega_;
    double d2 = pow(d,2);
    double L2 = pow(L,2);
    double J2 = pow(J,2);
    double R2 = pow(R,2);
    double Km2 = pow(Km,2);
    double Km3 = Km2 * Km;
    double Om = sqrt(d2*L2 + J2*R2 - 2*J*L*(2*Km2 + d*R));
    double eOp1 = exp((Om*dt)/(J*L)) + 1.0;
    double eOm1 = eOp1 - 2.0; // = exp((Om*t)/(J*L)) - 1.0;
    double eA = exp(((d*L + Om + J*R)*dt)/(2.0*J*L));
    double emA = 1.0/eA; // = exp(-((d*L + Om + J*R)*t)/(2.0*J*L));
    double i_t = (emA*(i0*(Km2 + d*R)*(d*L*(d*eOp1*L + eOm1*Om) + eOp1*J2*R2 - J*(4*eOp1*Km2*L + 2*d*eOp1*L*R + eOm1*Om*R)) - d*L*(d*(-2*eA + eOp1)*L + eOm1*Om)*(Km*T + d*V) - (-2*eA + eOp1)*J2*R2*(Km*T + d*V) + J*(Km3*(-2*eOm1*o0*Om + 4*(-2*eA + eOp1)*L*T) - Km*R*(2*d*eOm1*o0*Om - 2*d*(-2*eA + eOp1)*L*T + eOm1*Om*T) + 2*Km2*(2*d*(-2*eA + eOp1)*L + eOm1*Om)*V + d*(2*d*(-2*eA + eOp1)*L + eOm1*Om)*R*V)))/ (2.*(Km2 + d*R)*(d2*L2 + J2*R2 - 2*J*L*(2*Km2 + d*R)));
    double o_t = (emA*(-4*eOp1*J*pow(Km,4)*L*o0 + J*Km2*R* (-6*d*eOp1*L*o0 + eOm1*o0*Om - 4*(-2*eA + eOp1)*L*T) + J*R2*(-2*d2*eOp1*L*o0 + d*eOm1*o0*Om - 2*d*(-2*eA + eOp1)*L*T + eOm1*Om*T) + 4*(-2*eA + eOp1)*J*Km3*L*V - J*Km*(-2*d*(-2*eA + eOp1)*L + eOm1*Om)*R*V + J2*R2*(eOp1*Km2*o0 + d*eOp1*o0*R + (-2*eA + eOp1)*R*T - (-2*eA + eOp1)*Km*V) + L*(pow(d,3)*eOp1*L*o0*R + 2*eOm1*Km2*Om*(i0*Km - T) + d2*(eOp1*Km2*L*o0 - eOm1*o0*Om*R + (-2*eA + eOp1)*L*R*T -  (-2*eA + eOp1)*Km*L*V) - d*eOm1*Om*(Km2*o0 + R*T + Km*(-2*i0*R + V)) )))/(2.*(Km2 + d*R)* (d2*L2 + J2*R2 - 2*J*L*(2*Km2 + d*R)));
    // Update internal variables
    internal_current_ = i_t;
    internal_omega_   = o_t;
    ignition::math::Vector3d applied_torque;
    applied_torque.Z() = Km * i_t * gear_ratio_; // motor torque T_ext = K * i * n_gear
    this->link_->AddRelativeTorque(applied_torque);
}

// Plugin update function
void GazeboRosMotor::UpdateChild() {
    common::Time current_time = parent->GetWorld()->SimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    physics::JointWrench current_wrench;
    current_wrench = joint_->GetForceTorque( 0u );
    double current_output_speed = joint_->GetVelocity( 0u );
    motorModelUpdate(seconds_since_last_update, current_output_speed, current_wrench);

    if ( seconds_since_last_update > update_period_ ) {
        publishWheelJointState();
        publishJointWrench ( current_wrench, current_time );
        publishMotorCurrent();
        publishRotorVelocity( current_output_speed );
        publishEncoderCount( current_output_speed , seconds_since_last_update );
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
