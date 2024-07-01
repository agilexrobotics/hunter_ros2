/*
 * hunter_base_ros.cpp
 *
 * Created on: Oct 15, 2021 14:35
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "hunter_base/hunter_base_ros.hpp"

#include "hunter_base/hunter_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
HunterBaseRos::HunterBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  this->declare_parameter("port_name", rclcpp::ParameterValue("can0"));   //声明参数

  this->declare_parameter("odom_frame", rclcpp::ParameterValue("odom"));
  this->declare_parameter("base_frame", rclcpp::ParameterValue("base_link"));
  this->declare_parameter("odom_topic_name", rclcpp::ParameterValue("odom"));

  this->declare_parameter("is_hunter_mini", rclcpp::ParameterValue(false));
  this->declare_parameter("is_omni_wheel", rclcpp::ParameterValue(false));

  this->declare_parameter("simulated_robot", rclcpp::ParameterValue(false));
  this->declare_parameter("control_rate", rclcpp::ParameterValue(50));

  
  this->declare_parameter("enable_pd_regulator", true);
  this->declare_parameter("kp_v", 1.0);
  this->declare_parameter("kd_v", 0.1);
  this->declare_parameter("kp_w", 1.0);
  this->declare_parameter("kd_w", 0.1);

  LoadParameters();
}

void HunterBaseRos::LoadParameters() {
  this->get_parameter_or<std::string>("port_name", port_name_, "can0");//获取参数

  this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
  this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
  this->get_parameter_or<std::string>("odom_topic_name", odom_topic_name_, "odom");
  this->get_parameter_or<bool>("simulated_robot", simulated_robot_, false);
  this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);

  enable_pd_regulator_ = this->get_parameter("enable_pd_regulator").as_bool();
  kp_v_ = this->get_parameter("kp_v").as_double();
  kd_v_ = this->get_parameter("kd_v").as_double();
  kp_w_ = this->get_parameter("kp_w").as_double();
  kd_w_ = this->get_parameter("kd_w").as_double();

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- odom frame name: " << odom_frame_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;
  std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;

  std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
            << std::endl;
  std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool HunterBaseRos::Initialize() {

  std::cout << "Robot base: Hunter" << std::endl;
  ProtocolDetector detector;
  if (detector.Connect(port_name_)) {
    auto proto = detector.DetectProtocolVersion(5);
    if (proto == ProtocolVersion::AGX_V1) {
      std::cout << "Detected protocol: AGX_V1" << std::endl;
      robot_ = std::unique_ptr<HunterRobot>(new HunterRobot(ProtocolVersion::AGX_V1));
        version = 1;
    } else if (proto == ProtocolVersion::AGX_V2) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
         robot_ = std::unique_ptr<HunterRobot>(
              new HunterRobot(ProtocolVersion::AGX_V2));
        version = 2;
    } else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return false;
    }
  } else {
    return false;
  }

  return true;
}

void HunterBaseRos::Stop() { keep_running_ = false; }

void HunterBaseRos::Run() {

  // instantiate a ROS messenger
    std::unique_ptr<HunterMessenger<HunterRobot>> messenger =
        std::unique_ptr<HunterMessenger<HunterRobot>>(
            new HunterMessenger<HunterRobot>(robot_, this));

    if(version == 1)
    {
      messenger->SetTrack(HunterV1Params::track);
      messenger->SetWeelbase(HunterV1Params::wheelbase);
      messenger->SetMaxSteerAngleCentral(HunterV1Params::max_steer_angle_central);
      messenger->SetMaxSteerAngle(HunterV1Params::max_steer_angle);
      std::cout<< "========== Vehicle params V1 ================"  << std::endl;
      std::cout<< " track: " << HunterV1Params::track  << std::endl;
      std::cout<< " wheelbase: " << HunterV1Params::wheelbase  << std::endl;
      std::cout<< " max_steer_angle_central: " << HunterV1Params::max_steer_angle_central  << std::endl;
      std::cout<< " max_steer_angle: " << HunterV1Params::max_steer_angle  << std::endl;
    }
    else
    {
      messenger->SetTrack(HunterV2Params::track);
      messenger->SetWeelbase(HunterV2Params::wheelbase);
      messenger->SetMaxSteerAngleCentral(HunterV2Params::max_steer_angle_central);
      messenger->SetMaxSteerAngle(HunterV2Params::max_steer_angle);
      std::cout<< "========== Vehicle params V2 ================"  << std::endl;
      std::cout<< " track: " << HunterV2Params::track  << std::endl;
      std::cout<< " wheelbase: " << HunterV2Params::wheelbase  << std::endl;
      std::cout<< " max_steer_angle_central: " << HunterV2Params::max_steer_angle_central  << std::endl;
      std::cout<< " max_steer_angle: " << HunterV2Params::max_steer_angle  << std::endl;
    }

    messenger->SetRegulatorParams(kp_v_, kd_v_, kp_w_, kd_w_, enable_pd_regulator_);
    messenger->SetOdometryFrame(odom_frame_);
    messenger->SetBaseFrame(base_frame_);
    messenger->SetOdometryTopicName(odom_topic_name_);
    if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

    // connect to robot and setup ROS subscription
    if (port_name_.find("can") != std::string::npos) {
      if (robot_->Connect(port_name_)) {
        robot_->EnableCommandedMode();
        // std::cout << "EnableCommandedMode" << std::endl;
        std::cout << "Using CAN bus to talk with the robot" << std::endl;
      } else {
        std::cout << "Failed to connect to the robot CAN bus" << std::endl;
        return;
      }
    } else {
      std::cout << "Please check the specified port name is a CAN port"
                << std::endl;
      return;
    }

    // publish robot state at 50Hz while listening to twist commands
    messenger->SetupSubscription();
    rclcpp::Rate rate(50);
    keep_running_ = true;
    while (keep_running_) {
      messenger->PublishStateToROS();
      // robot_->EnableCommandedMode();
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    // }
  }
}
}  // namespace westonrobot
