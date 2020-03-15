# Gazebo ROS motor plugins

This repository contains currently <u>two</u> motor plugins, one with an ideal speed controller and one without a controller that models a DC motor. *Planning to extend the package with AC, BLDC, stepper and servo motor models as well...*

![](doc/plugins.png)

## DC motor plugin

This plugin is trying to estimate a parametric DC motor by the solution of it's electro-mechanical differential equations, exposing the virtual motor's current consumption and torque interfaces as topics. It has been designed to allow motor control system development in ROS nodes utilizing the physics engine of Gazebo.

![](doc/dc_curves.png)

### Electro-mechanical model

![](doc/MCC.png)

![](doc/variables.png)

From Kirchoff's voltage law:

![](doc/kirchoff.svg)

By Newton's law:

![](doc/newton.svg)

Electro-mechanical coupling:

![](doc/back_emf.svg)

![](doc/torque.svg)

Where ![](doc/ke.svg) is the electromotive constant and back EMF (e) is proportional to the shaft angular velocity, and ![](doc/kt.svg) is the torque constant while the torque (T) is proportional to actual motor current. The mechanical power produced by the DC motor is ![T_m\omega = K_Ti\omega](doc/mech_pwr.svg). The electric power ![P_e = vi](doc/elec_pwr.svg) delivered by the source goes into heat loss in the resistance ![R](doc/res.svg), into stored magnetic energy in the inductance ![L](doc/ind.svg) and the remaining quantity ![iK_e\omega](doc/rem.svg) is converted in mechanical energy ![T_m\omega](doc/mech_en.svg). It leads to ![T_m\omega = K_Ti\omega = K_ei\omega](doc/teq.svg) whether ![K_T = K_e = K_\phi](doc/keq.svg) *( Chiasson 2005 ).* *Model description by [INSA Lyon.](http://www.ctrl-elec.fr/en/ctrl-elec/motor-control/dc-motor-control/84-2/)*

### Simulator interface

The controlled input for the DC motor model is the voltage that is applied to the motor's electrical connectors, environmental inputs are the shaft velocity and the external load torque on the shaft. Output is the electromotive force produced by the virtual motor.

```c++
link_->AddTorque(ignition::math::Vector3d applied_torque); // apply torque
joint_->GetForceTorque( 0u ); // get external load
joint_->GetVelocity( 0u ); // get shaft angular velocity
```

#### Advantages of applying torque on the link

- Supported on all physics engines
- Object feels the force that accelerates it
- Can undershoot or overshoot the target velocity *( ideal for controller development )*
- Shaft's damping and friction can be set as joint properties

### Usage

Just use the **dc_motor** macro in a descriptor file as if it were a joint:

```xml
  <xacro:include filename="$(find gazebo_ros_motors)/xacro/dc_motor.xacro"/>
  <xacro:dc_motor motor_name="dc_motor" parent_link="base_link" child_link="wheel_link">
    <xacro:property name="params_yaml" value="$(find gazebo_ros_motors)/params/dc_motor.yaml"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </xacro:dc_motor>
```

Plugin parameters are loaded from a **yaml** file *( defaults are close to a 24V / 450W wheelchair motor )*:

```yaml
# motor model
motor_nominal_voltage: 24.0 # Volts
moment_of_inertia: 0.001 # kgm^2
armature_damping_ratio: 0.0001 # Nm/(rad/s)
electromotive_force_constant: 0.08 # Nm/A
electric_resistance: 1.0 # Ohm
electric_inductance: 0.001 # Henry
# transmission
gear_ratio: 20.0
joint_damping: 0.005
joint_friction: 0.01
# shaft encoder
encoder_ppr: 4096
```

#### ROS Topics

**Subscriber**

- **/motor/command**  -- normalized voltage, scales to **motor_nominal_voltage** parameter

**Publishers**

- **/motor/velocity** -- motor shaft velocity *(encoder side, before gearbox)* **( rad / sec )**
- **/motor/encoder** -- encoder counter 
- **/motor/wrench** -- joint's torque and force
- **/motor/current** -- electrical current flowing on the coil



## ODE joint motor plugin

This plugin is using the Open Dynamics Engine's joint motor interface, that implements an ideal speed controller. The reason behind a single velocity encoded motor was to allow the development of parametric multi-actuated kinematic models *(E.g. 3 omni wheel geometry )* without the need to tune a controller.![](doc/ode_controlled.png)

### Simulator interface

The plugin's input is the desired angular velocity of the motor joint, and the maximum force the ODE joint-motor may apply during a time step as parameter. 

```c++
joint_->SetParam("fmax", 0, motor_fmax_); // set max. force applied per time step
joint_->SetParam("vel",  0, input_); // set shaft angular velocity 
joint_->GetVelocity( 0u )*encoder_to_shaft_ratio_; // read joint velocity 
```

### Usage

Just use the **joint_motor** macro in a descriptor file as if it were a joint:

```xml
  <xacro:include filename="$(find gazebo_ros_motors)/xacro/joint_motor.xacro"/>
  <xacro:joint_motor motor_name="ode_motor" parent_link="base_link" child_link="wheel_link">
    <xacro:property name="params_yaml" value="$(find gazebo_ros_motors)/params/joint_motor.yaml"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </xacro:joint_motor>
```

Plugin parameters are loaded from a **yaml** file:

```yaml
# ODE joint motor
ode_joint_motor_fmax: 50.0
ode_joint_fudge_factor: 1.0
# transmission
joint_damping: 0.005
joint_friction: 0.01
# shaft encoder
encoder_ppr: 4096
encoder_to_shaft_ratio: 1.0
```

More details on *fmax* and *fudge factor* can be found in the [ODE user guide](https://ode.org/ode-latest-userguide.html#sec_7_5_0). 

#### ROS Topics

**Subscriber**

- **/motor/command**  -- desired joint angular velocity **( rad / sec )**

**Publishers**

- **/motor/velocity** -- motor shaft velocity *(encoder side, before gearbox)* **( rad / sec )**
- **/motor/encoder** -- encoder counter 





##### Authors

Marton Yuhaas [nilseuropa](https://github.com/nilseuropa) and Gergely Gyebroszki [gyebro](https://github.com/Gyebro)