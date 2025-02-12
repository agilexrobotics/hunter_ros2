/*
 * hunter_messenger.hpp
 *
 * Created on: Jun 14, 2019 10:24
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_MESSENGER_HPP
#define HUNTER_MESSENGER_HPP

#include <string>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ascent/Ascent.h"
#include "ascent/Utility.h"
#include "hunter_base/bicycle_model.hpp"
#include "hunter_base/hunter_params.hpp"

#include "ugv_sdk/mobile_robot/hunter_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include "hunter_msgs/msg/hunter_status.hpp"
#include "hunter_msgs/msg/hunter_light_cmd.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

namespace westonrobot {

template <typename SystemModel>
class SystemPropagator {
 public:
  asc::state_t Propagate(asc::state_t init_state,
                         typename SystemModel::control_t u, double t0,
                         double tf, double dt) {
    double t = t0;
    asc::state_t x = init_state;

    while (t <= tf) {
      integrator_(SystemModel(u), x, t, dt);
      // Note: you may need to add additional constraints to [x]
    }
    return x;
  }

 private:
  asc::RK4 integrator_;
};

template <typename HunterType>
class HunterMessenger {
 public:
  HunterMessenger(std::shared_ptr<HunterType> hunter, rclcpp::Node *node)
      : hunter_(hunter), node_(node) {}
  // SystemPropagator<BicycleKinematics> model_;
  void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
  void SetBaseFrame(std::string frame) { base_frame_ = frame; }
  void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }
  void SetWeelbase(float Weelbase){
    l = Weelbase;
  }
  void SetTrack(float Track){
    w = Track;
  }
  void SetMaxSteerAngleCentral(float Angle){
    max_steer_angle_central = Angle;
  }
  void SetMaxSteerAngle(float Angle){
    max_steer_angle = Angle;
  }
  void SetSimulationMode(int loop_rate) {
    simulated_robot_ = true;
    sim_control_rate_ = loop_rate;
  }
  
  // Parameters:
  // - kp_v: Proportional gain for linear velocity
  // - kd_v: Derivative gain for linear velocity
  // - kp_w: Proportional gain for angular velocity
  // - kd_w: Derivative gain for angular velocity
  // - enable_pd_regulator: Boolean flag to enable or disable the PD regulator
  void SetRegulatorParams(double kp_v, double kd_v, double kp_w, double kd_w, bool enable_pd_regulator) {
    kp_v_ = kp_v;
    kd_v_ = kd_v;
    kp_w_ = kp_w;
    kd_w_ = kd_w;
    enable_pd_regulator_ = enable_pd_regulator;
  }

  void SetupSubscription() {
    // odometry publisher
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, qos_profile);

    // joint state publisher
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    status_pub_ = node_->create_publisher<hunter_msgs::msg::HunterStatus>(
        "/hunter_status", 10);

    // cmd subscriber
    motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/hunter/cmd_vel", 10,
        std::bind(&HunterMessenger::TwistCmdCallback, this,
                  std::placeholders::_1));

    if(enable_pd_regulator_){

      odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/hunter/global_odom", rclcpp::SensorDataQoS().keep_last(1)
            , std::bind(&HunterMessenger::odometryCallback, this, std::placeholders::_1));

      parameter_event_sub_ = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
              "/parameter_events", 10, std::bind(&HunterMessenger::onParameterEvent, this, std::placeholders::_1));

    }
  }

  void PublishStateToROS() {
    current_time_ = node_->get_clock()->now();
    static bool init_run = true;
    if (init_run) {
      last_time_ = current_time_;
      last_time_control_law_ = node_->get_clock()->now();
      init_run = false;
      return;
    }
    double dt = (current_time_ - last_time_).seconds();

    auto state = hunter_->GetRobotState();

    // publish hunter state message
    hunter_msgs::msg::HunterStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = state.motion_state.linear_velocity;
    double phi =ConvertInnerAngleToCentral(state.motion_state.steering_angle);
    status_msg.steering_angle = phi;
    // status_msg.angular_velocity = state.motion_state.steering_angle;

    status_msg.vehicle_state = state.system_state.vehicle_state;
    status_msg.control_mode = state.system_state.control_mode;
    status_msg.error_code = state.system_state.error_code;
    status_msg.battery_voltage = state.system_state.battery_voltage;

    auto actuator = hunter_->GetActuatorState();

    for (int i = 0; i < 3; ++i) {
      // actuator_hs_state
      uint8_t motor_id = actuator.actuator_hs_state[i].motor_id;

      status_msg.actuator_states[motor_id].rpm =
          actuator.actuator_hs_state[i].rpm;
      status_msg.actuator_states[motor_id].current =
          actuator.actuator_hs_state[i].current;
      status_msg.actuator_states[motor_id].pulse_count =
          actuator.actuator_hs_state[i].pulse_count;

      // actuator_ls_state
      motor_id = actuator.actuator_ls_state[i].motor_id;

      status_msg.actuator_states[motor_id].driver_voltage =
          actuator.actuator_ls_state[i].driver_voltage;
      status_msg.actuator_states[motor_id].driver_temperature =
          actuator.actuator_ls_state[i].driver_temp;
      status_msg.actuator_states[motor_id].motor_temperature =
          actuator.actuator_ls_state[i].motor_temp;
      status_msg.actuator_states[motor_id].driver_state =
          actuator.actuator_ls_state[i].driver_state;
    }

    status_pub_->publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(state.motion_state, dt);

    // --- New Code: Publish Joint States ---
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = current_time_;

    // Set joint names as defined in your URDF/config
    joint_state_msg.name.push_back("rear_left_wheel_joint");     // Traction joint
    joint_state_msg.name.push_back("front_left_steering_joint");   // Steering joint

    //convert linear velocity to position
    static double wheel_position = 0.0;
    wheel_position += state.motion_state.linear_velocity * dt;
    double steering_position = state.motion_state.steering_angle;     // Or convert if needed

    joint_state_msg.position.push_back(wheel_position);
    joint_state_msg.position.push_back(steering_position);

    // Optionally, if you have velocities:
    // joint_state_msg.velocity.push_back(state.motion_state.linear_velocity);

    joint_state_pub_->publish(joint_state_msg);

    // record time for next integration
    last_time_ = current_time_;
  }

 private:
  std::shared_ptr<HunterType> hunter_;
  rclcpp::Node *node_;
  

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  westonrobot::SystemPropagator<BicycleKinematics> model_;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<hunter_msgs::msg::HunterStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  

  // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // speed variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  static constexpr double steer_angle_tolerance = 0.005;  // ~+-0.287 degrees
  double l = 0.0;
  double w = 0.0;
  double max_steer_angle_central = 0.0;
  double max_steer_angle = 0.0;

  rclcpp::Time last_time_;
  rclcpp::Time last_time_control_law_;
  rclcpp::Time current_time_;

  double kp_v_, kd_v_, kp_w_, kd_w_;
  bool enable_pd_regulator_;
  double v_, omega_;
  double v_d_, omega_d_;
  double previous_error_v_, previous_error_omega_;


  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    
    v_d_ = msg->linear.x;
    omega_d_ = msg->angular.z;
   
    if (!simulated_robot_) {
      SetHunterMotionCommand(msg);
    } else {
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("Cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        v_ = msg->twist.twist.linear.x;
        omega_ = msg->twist.twist.angular.z;
  }

  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
      for (auto & changed_parameter : event->changed_parameters) {
          const std::string & name = changed_parameter.name;
          if (name == "kp_v") {
              kp_v_ = changed_parameter.value.double_value;
              RCLCPP_INFO(node_->get_logger(), "Updated kp_v to: %f", kp_v_);
          } else if (name == "kd_v") {
              kd_v_ = changed_parameter.value.double_value;
              RCLCPP_INFO(node_->get_logger(), "Updated kd_v to: %f", kd_v_);
          } else if (name == "kp_w") {
              kp_w_ = changed_parameter.value.double_value;
              RCLCPP_INFO(node_->get_logger(), "Updated kp_w to: %f", kp_w_);
          } else if (name == "kd_w") {
              kd_w_ = changed_parameter.value.double_value;
              RCLCPP_INFO(node_->get_logger(), "Updated kd_w to: %f", kd_w_);
          }
      }
  }


  // define the control law
  void controlLoop(double& len_vel, double& anu_vel) {

    auto current_time = node_->get_clock()->now();
    double dt = 1.0/sim_control_rate_;
    // double dt = (current_time - last_time_control_law_).seconds();
    last_time_control_law_ = current_time;

    if (dt < 0.0) {
        len_vel = 0.0;
        anu_vel = 0.0;
        return;
    }

    // RCLCPP_INFO(node_->get_logger(), "control v_d_ :%f, v_ %f omega_d_ :%f, omega_ %f ", v_d_, v_, omega_d_, omega_);

    // compute the velocity and angular velocity errors
    double error_v = v_d_ - v_;
    double error_omega = omega_d_ - omega_;
    
    // derivative of error (assuming discrete time implementation)
    double error_dot_v = (error_v - previous_error_v_) / dt;
    double error_dot_omega = (error_omega - previous_error_omega_) / dt;

    // RCLCPP_INFO(node_->get_logger(), "error_v :%f, v_ %f error_omega :%f, error_dot_omega %f ", error_v, error_omega, error_dot_v, error_dot_omega);
    
    // PD control laws
    double a = kp_v_ * error_v + kd_v_ * error_dot_v;
    double alpha = kp_w_ * error_omega + kd_w_ * error_dot_omega;

    // RCLCPP_INFO(node_->get_logger(), "a:%f, alpha %f  v_ %f ", a, alpha, v_);
    
    // Update the velocities
    
    v_ += a * dt;
    omega_ += alpha * dt;
    // RCLCPP_INFO(node_->get_logger(), "v_ %f, omega %f ", v_, omega_);
    
    // Update the previous errors
    previous_error_v_ = error_v;
    previous_error_omega_ = error_omega;

    len_vel = v_;
    anu_vel = omega_;

  }


  // template <typename T,std::enable_if_t<!std::is_base_of<HunterRobot, T>::value,bool> = true>
  void SetHunterMotionCommand(const geometry_msgs::msg::Twist::SharedPtr &msg) {
    std::shared_ptr<HunterRobot> base;
    double radian = 0.0;
    double phi_i = 0.0;
    if(enable_pd_regulator_){
      double len_vel, anu_vel;
      controlLoop(len_vel, anu_vel);
      phi_i = AngelVelocity2Angel(len_vel, anu_vel, radian);
      hunter_->SetMotionCommand(len_vel, phi_i);
    } else{
      phi_i = AngelVelocity2Angel(*msg, radian);
      hunter_->SetMotionCommand(msg->linear.x, phi_i);
    }
    // std::cout << "set steering angle: " << phi_i << std::endl;
  }

  double ConvertCentralAngleToInner(double angle)
  {
    double phi = angle;
    double phi_i = 0;
    if (phi > steer_angle_tolerance) {
      // left turn
      phi_i = std::atan(2 * l * std::sin(phi) /
                        (2 * l * std::cos(phi) - w * std::sin(phi)));
    } else if (phi < -steer_angle_tolerance) {
      // right turn
      phi = -phi;
      phi_i = std::atan(2 * l * std::sin(phi) /
                        (2 * l * std::cos(phi) - w * std::sin(phi)));
      phi_i = -phi_i;
    }
    return phi_i;
  }

  double ConvertInnerAngleToCentral(double angle)
  {
    double phi = 0;
    double phi_i = angle;
    if (phi_i > steer_angle_tolerance) {
      // left turn
      double r = l / std::tan(phi_i) + w / 2;
      phi = std::atan(l / r);
    } else if (phi_i < -steer_angle_tolerance) {
      // right turn
      double r = l / std::tan(-phi_i) + w / 2;
      phi = std::atan(l / r);
      phi = -phi;
    }
    return phi;

  }

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  void PublishOdometryToROS(const MotionStateMessage &msg, double dt) {
    // perform numerical integration to get an estimation of pose
    double linear_speed_ = msg.linear_velocity;
    // double angular_speed = msg.angular_velocity;
    double steering_angle_ = ConvertInnerAngleToCentral(msg.steering_angle);
    // double lateral_speed = 0;

    asc::state_t state =
      model_.Propagate({position_x_, position_y_, theta_},
                       {linear_speed_, steering_angle_}, 0, dt, dt / 100);
    position_x_ = state[0];
    position_y_ = state[1];
    theta_ = state[2];


    geometry_msgs::msg::Quaternion odom_quat =
        createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    // tf_broadcaster_->sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = linear_speed_ / l * std::tan(steering_angle_);

    odom_msg.pose.covariance = {
    0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
    0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
    0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
    0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
    0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
    0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};
    odom_msg.twist.covariance = {
    0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
    0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
    0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
    0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
    0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
    0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};

    odom_pub_->publish(odom_msg);
  }
  
  double AngelVelocity2Angel(geometry_msgs::msg::Twist msg,double &radius)
  {
    return AngelVelocity2Angel(msg.linear.x, msg.angular.z, radius);
  }

  double AngelVelocity2Angel(double& linear_vel, double& angular_vel, double &radius)
  {
    double linear = fabs(linear_vel);
    double angular = fabs(angular_vel);

    if(angular < 0.001) // adding dead zone
    {
      return 0.0;
    }

    radius = linear / angular;

    int k = angular_vel / fabs(angular_vel);
    if ((radius-l)<0 )
    {
      return  k*max_steer_angle;
    }

    double phi_i;
    phi_i = atan(l/(radius-w/2));
    if(linear_vel<0)
      phi_i *= -1.0;
    return k*phi_i;
  }
};
}  // namespace westonrobot

#endif /* Hunter_MESSENGER_HPP */
