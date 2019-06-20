#include <quadrotor_sliding_controller.h>
#include "sdf/sdf.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/physics/physics.hh"

#include <cmath>
#include <stdlib.h>

namespace gazebo {

GazeboQuadrotorSlidingController::GazeboQuadrotorSlidingController()
{

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboQuadrotorSlidingController::~GazeboQuadrotorSlidingController()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboQuadrotorSlidingController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();



  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>("") +"/";


  if (!_sdf->HasElement("topicName"))
    velocity_topic_ = "cmd_vel";
  else
    velocity_topic_ = _sdf->GetElement("topicName")->Get<std::string>("");


  if (!_sdf->HasElement("imuTopic"))
    imu_topic_.clear();
  else
    imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>("");

  if (!_sdf->HasElement("stateTopic"))
    state_topic_.clear();
  else
    state_topic_ = _sdf->GetElement("stateTopic")->Get<std::string>("");

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>("");
    link = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link_name_));
  }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("maxForce"))
    max_force_ = -1;
  else
    max_force_ = _sdf->GetElement("maxForce")->Get<double>("");


  if (!_sdf->HasElement("motionSmallNoise"))
    motion_small_noise_ = 0;
  else
    motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>("");

  if (!_sdf->HasElement("motionDriftNoise"))
    motion_drift_noise_ = 0;
  else
    motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>("");

  if (!_sdf->HasElement("motionDriftNoiseTime"))
    motion_drift_noise_time_ = 1.0;
  else
    motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>("");

    if (!_sdf->HasElement("publishFrequency"))
      publish_frequency_ = 200;
    else
      publish_frequency_ = _sdf->GetElement("publishFrequency")->Get<double>("");


  controllers.velocity_x.Load(_sdf, "velocityXY");
  controllers.velocity_y.Load(_sdf, "velocityXY");
  controllers.velocity_z.Load(_sdf, "velocityZ");
  controllers.attitude.Load(_sdf, "attitude");

  // Get inertia and mass of quadrotor body
  inertia = link->GetInertial()->GetPrincipalMoments();
  mass = link->GetInertial()->GetMass();

  controllers.attitude.inertia = inertia;

  node_handle_ = new ros::NodeHandle(namespace_);

  // subscribe command: velocity control command
  if (!velocity_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      velocity_topic_, 1,
      boost::bind(&GazeboQuadrotorSlidingController::VelocityCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    velocity_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe imu
  if (!imu_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
      imu_topic_, 1,
      boost::bind(&GazeboQuadrotorSlidingController::ImuCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    imu_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("quadrotor_sliding_controller", "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
  }

  // subscribe state
  if (!state_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(
      state_topic_, 1,
      boost::bind(&GazeboQuadrotorSlidingController::StateCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    state_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("quadrotor_sliding_controller", "Using state information on topic %s as source of state information.", state_topic_.c_str());
  }

  torque_publisher_ = node_handle_->advertise<geometry_msgs::Vector3>("/" + namespace_+"/torque",100);
  att_command_publisher_ = node_handle_->advertise<geometry_msgs::Vector3>("/" + namespace_+"/att_cmd",100);
  att_publisher_ = node_handle_->advertise<geometry_msgs::Vector3>("/" + namespace_+"/att",100);
  // callback_queue_thread_ = boost::thread( boost::bind( &GazeboQuadrotorSlidingController::CallbackQueueThread,this ) );


  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboQuadrotorSlidingController::Update, this));

  ROS_INFO("Sliding controller loaded");
  publish_time = world->GetSimTime();


}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void GazeboQuadrotorSlidingController::VelocityCallback(const geometry_msgs::TwistConstPtr& velocity)
{
  velocity_command_ = *velocity;


  static common::Time last_sim_time = world->GetSimTime();
  static double time_counter_for_drift_noise = 0;
  static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
  // Get simulator time
  common::Time cur_sim_time = world->GetSimTime();
  double dt = (cur_sim_time - last_sim_time).Double();
  // save last time stamp
  last_sim_time = cur_sim_time;

  // generate noise
  if(time_counter_for_drift_noise > motion_drift_noise_time_)
  {
    drift_noise[0] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[1] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[2] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[3] = 2*motion_drift_noise_*(drand48()-0.5);
    time_counter_for_drift_noise = 0.0;
  }
  time_counter_for_drift_noise += dt;


  velocity_command_.linear.x += drift_noise[0] + 2*motion_small_noise_*(drand48()-0.5);
  velocity_command_.linear.y += drift_noise[1] + 2*motion_small_noise_*(drand48()-0.5);
  velocity_command_.linear.z += drift_noise[2] + 2*motion_small_noise_*(drand48()-0.5);
  velocity_command_.angular.z += drift_noise[3] + 2*motion_small_noise_*(drand48()-0.5);

}

void GazeboQuadrotorSlidingController::ImuCallback(const sensor_msgs::ImuConstPtr& imu)
{
  pose.rot.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.rot.GetAsEuler();
  angular_velocity = pose.rot.RotateVector(math::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
}

void GazeboQuadrotorSlidingController::StateCallback(const nav_msgs::OdometryConstPtr& state)
{
  math::Vector3 velocity1(velocity);

  if (imu_topic_.empty()) {
    pose.pos.Set(state->pose.pose.position.x, state->pose.pose.position.y, state->pose.pose.position.z);
    pose.rot.Set(state->pose.pose.orientation.w, state->pose.pose.orientation.x, state->pose.pose.orientation.y, state->pose.pose.orientation.z);
    euler = pose.rot.GetAsEuler();
    angular_velocity.Set(state->twist.twist.angular.x, state->twist.twist.angular.y, state->twist.twist.angular.z);
  }

  velocity.Set(state->twist.twist.linear.x, state->twist.twist.linear.y, state->twist.twist.linear.z);

  // calculate acceleration
  double dt = !state_stamp.isZero() ? (state->header.stamp - state_stamp).toSec() : 0.0;
  state_stamp = state->header.stamp;
  if (dt > 0.0) {
    acceleration = (velocity - velocity1) / dt;
  } else {
    acceleration.Set();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboQuadrotorSlidingController::Update()
{
  math::Vector3 force, torque;

  // Get new commands/state
  callback_queue_.callAvailable();

  // Get simulator time
  common::Time sim_time = world->GetSimTime();
  double dt = (sim_time - last_time).Double();
  if (dt == 0.0) return;

  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  if (imu_topic_.empty()) {
    pose = link->GetWorldPose();
    angular_velocity = link->GetWorldAngularVel();
    euler = pose.rot.GetAsEuler();
  }
  if (state_topic_.empty()) {
    acceleration = (link->GetWorldLinearVel() - velocity) / dt;
    velocity = link->GetWorldLinearVel();
  }

  // Get gravity
  math::Vector3 gravity_body = pose.rot.RotateVector(world->GetPhysicsEngine()->GetGravity());
  double gravity = gravity_body.GetLength();
  double load_factor = gravity * gravity / world->GetPhysicsEngine()->GetGravity().Dot(gravity_body);  // Get gravity

  // Rotate vectors to coordinate frames relevant for control
  math::Quaternion heading_quaternion(cos(euler.z/2),0,0,sin(euler.z/2));
  math::Vector3 velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
  math::Vector3 acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);

  math::Vector3 angular_velocity_body = pose.rot.RotateVectorReverse(angular_velocity);

  // update controllers
  force.Set(0.0, 0.0, 0.0);
  torque.Set(0.0, 0.0, 0.0);
  math::Vector3 attitude_command;
  attitude_command.x = -controllers.velocity_y.update(velocity_command_.linear.y, velocity_xy.y, acceleration_xy.y, dt) / gravity;
  attitude_command.y =  controllers.velocity_x.update(velocity_command_.linear.x, velocity_xy.x, acceleration_xy.x, dt) / gravity;
  attitude_command.z = velocity_command_.angular.z;
  torque = controllers.attitude.update(attitude_command, euler, angular_velocity_body, dt);
  force.z  = mass * (controllers.velocity_z.update(velocity_command_.linear.z,  velocity.z, acceleration.z, dt) + load_factor * gravity);
  if (max_force_ > 0.0 && force.z > max_force_) force.z = max_force_;
  if (force.z < 0.0) force.z = 0.0;

  link->AddRelativeForce(force);
  link->AddRelativeTorque(torque);

  if ((world->GetSimTime()-publish_time).Double() >= (1.0/publish_frequency_))
  {
    // Publish torque ,attitude_command & attitude
      geometry_msgs::Vector3 torque_msg;
      torque_msg.x=torque.x;
      torque_msg.y=torque.y;
      torque_msg.z=torque.z;
      torque_publisher_.publish(torque_msg);

      geometry_msgs::Vector3 att_com_msg;
      att_com_msg.x=attitude_command.x * (180/3.1415);
      att_com_msg.y=attitude_command.y * (180/3.1415);
      att_com_msg.z=attitude_command.z * (180/3.1415);
      att_command_publisher_.publish(att_com_msg);

      geometry_msgs::Vector3 att_msg;
      att_msg.x=euler.x * (180/3.1415);
      att_msg.y=euler.y * (180/3.1415);
      att_msg.z=euler.z * (180/3.1415);
      att_publisher_.publish(att_msg);

      publish_time = world->GetSimTime();

    }
  // ROS_INFO("Force %f \t Torque %f %f %f \n",force.z,torque.x,torque.y,torque.z);

  // save last time stamp
  last_time = sim_time;

}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboQuadrotorSlidingController::Reset()
{
  controllers.velocity_x.reset();
  controllers.velocity_y.reset();
  controllers.velocity_z.reset();
  controllers.attitude.reset();

  link->SetForce(math::Vector3(0,0,0));
  link->SetTorque(math::Vector3(0,0,0));

  // reset state
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  state_stamp = ros::Time();
}

////////////////////////////////////////////////////////////////////////////////
// Sliding controller implementation
GazeboQuadrotorSlidingController::SlidingController::SlidingController()
{
}

GazeboQuadrotorSlidingController::SlidingController::~SlidingController()
{
}

void GazeboQuadrotorSlidingController::SlidingController::Load(sdf::ElementPtr _sdf, const std::string& prefix)
{
  lambda1 = 0.50;
  lambda2 = 0.70;
  lambda3 = 1.00;

  mu1 = 0.01;
  mu2 = 0.03;
  mu3 = 0.06;

  time_constant = 0.0;
  limit = -1.0;

  if (!_sdf) return;


  // _sdf->PrintDescription(_sdf->GetName());
  if (_sdf->HasElement(prefix + "Lambda1")) lambda1 = _sdf->GetElement(prefix + "Lambda1")->Get<double>("");
  if (_sdf->HasElement(prefix + "Lambda2")) lambda2 = _sdf->GetElement(prefix + "Lambda2")->Get<double>("");
  if (_sdf->HasElement(prefix + "Lambda3")) lambda3 = _sdf->GetElement(prefix + "Lambda3")->Get<double>("");

  if (_sdf->HasElement(prefix + "Mu1")) mu1 = _sdf->GetElement(prefix + "Mu1")->Get<double>("");
  if (_sdf->HasElement(prefix + "Mu2")) mu2 = _sdf->GetElement(prefix + "Mu2")->Get<double>("");
  if (_sdf->HasElement(prefix + "Mu3")) mu3 = _sdf->GetElement(prefix + "Mu3")->Get<double>("");

  if (_sdf->HasElement(prefix + "TimeConstant")) time_constant = _sdf->GetElement(prefix + "TimeConstant")->Get<double>("");
  if (_sdf->HasElement(prefix + "Limit")) limit = _sdf->GetElement(prefix + "Limit")->Get<double>("");
  if (_sdf->HasElement(prefix + "YawLimit")) yawLimit = _sdf->GetElement(prefix + "YawLimit")->Get<double>("");

}

math::Vector3 GazeboQuadrotorSlidingController::SlidingController::update(math::Vector3 new_input, math::Vector3 x, math::Vector3 dx, double dt)
{
  math::Vector3 new_dinput;
  // limit command
  if (limit > 0.0 && fabs(new_input.x) > limit) new_input.x = (new_input.x < 0 ? -1.0 : 1.0) * limit;
  if (limit > 0.0 && fabs(new_input.y) > limit) new_input.y = (new_input.y < 0 ? -1.0 : 1.0) * limit;
  if (yawLimit > 0.0 && fabs(new_input.z) > yawLimit) new_input.z = (new_input.z < 0 ? -1.0 : 1.0) * yawLimit;

  // filter command
  if (dt + time_constant > 0.0) {

    new_dinput.x = (new_input.x - input.x) / (dt + time_constant);
    new_dinput.y = (new_input.y - input.y) / (dt + time_constant);
    new_dinput.z = (new_input.z - input.z) / (dt + time_constant);

    ddinput.x = (new_dinput.x - dinput.x) / (dt + time_constant);
    ddinput.y = (new_dinput.y - dinput.y) / (dt + time_constant);
    ddinput.z = (new_dinput.z - dinput.z) / (dt + time_constant);

    dinput = new_dinput;

    input.x  = (dt * new_input.x + time_constant * input.x) / (dt + time_constant);
    input.y  = (dt * new_input.y + time_constant * input.y) / (dt + time_constant);
    input.z  = (dt * new_input.z + time_constant * input.z) / (dt + time_constant);

  }

  ////////////////// Sliding Mode Control Law ////////////////////

  // S = (dinput - dx) + lambda * (input - x);
  double S1 = (dinput.x - dx.x) + lambda1 * (input.x - x.x);
  double S2 = (dinput.y - dx.y) + lambda2 * (input.y - x.y);
  double S3 = (dinput.z - dx.z) + lambda3 * (input.z - x.z);

  // update control output
  output.x = inertia.x*(ddinput.x + (inertia.z - inertia.y)*dx.y*dx.z + lambda1*dinput.x + mu1*tanh(S1) - lambda1*dx.x);
  output.y = inertia.y*(ddinput.y + (inertia.x - inertia.z)*dx.x*dx.z + lambda2*dinput.y + mu2*tanh(S2) - lambda2*dx.y);
  output.z = inertia.z*(ddinput.z + (inertia.y - inertia.x)*dx.y*dx.x + lambda3*dinput.z + mu3*tanh(S3) - lambda3*dx.z);

  // ROS_INFO(" Torque %f %f %f  \t S %f %f %f \n",output.x,output.y,output.z,S1,S2,S3);

  return output;
}

void GazeboQuadrotorSlidingController::SlidingController::reset()
{

}

///////////////////////////// PID Controller implementation //////////////////////////////

GazeboQuadrotorSlidingController::PIDController::PIDController()
{
}

GazeboQuadrotorSlidingController::PIDController::~PIDController()
{
}

void GazeboQuadrotorSlidingController::PIDController::Load(sdf::ElementPtr _sdf, const std::string& prefix)
{
  gain_p = 0.0;
  gain_d = 0.0;
  gain_i = 0.0;
  time_constant = 0.0;
  limit = -1.0;

  if (!_sdf) return;


  // _sdf->PrintDescription(_sdf->GetName());
  if (_sdf->HasElement(prefix + "ProportionalGain")) gain_p = _sdf->GetElement(prefix + "ProportionalGain")->Get<double>("");
  if (_sdf->HasElement(prefix + "DifferentialGain")) gain_d = _sdf->GetElement(prefix + "DifferentialGain")->Get<double>("");
  if (_sdf->HasElement(prefix + "IntegralGain"))     gain_i = _sdf->GetElement(prefix + "IntegralGain")->Get<double>("");
  if (_sdf->HasElement(prefix + "TimeConstant"))     time_constant = _sdf->GetElement(prefix + "TimeConstant")->Get<double>("");
  if (_sdf->HasElement(prefix + "Limit"))            limit = _sdf->GetElement(prefix + "Limit")->Get<double>("");

}
double GazeboQuadrotorSlidingController::PIDController::update(double new_input, double x, double dx, double dt)
{
  // limit command
  if (limit > 0.0 && fabs(new_input) > limit) new_input = (new_input < 0 ? -1.0 : 1.0) * limit;

  // filter command
  if (dt + time_constant > 0.0) {
    dinput = (new_input - input) / (dt + time_constant);
    input  = (dt * new_input + time_constant * input) / (dt + time_constant);
  }

  // update proportional, differential and integral errors
  p = input - x;
  d = dinput - dx;
  i = i + dt * p;

  // update control output
  output = gain_p * p + gain_d * d + gain_i * i;
  return output;
}
void GazeboQuadrotorSlidingController::PIDController::reset()
{
  input = dinput = 0;
  p = i = d = output = 0;
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboQuadrotorSlidingController)

} // namespace gazebo
