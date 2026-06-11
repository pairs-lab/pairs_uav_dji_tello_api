/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <pairs_uav_hw_api/api.h>

#include <pairs_lib/param_loader.h>
#include <pairs_lib/attitude_converter.h>
#include <pairs_lib/mutex.h>
#include <pairs_lib/publisher_handler.h>
#include <pairs_lib/subscriber_handler.h>
#include <pairs_lib/service_client_handler.h>

#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

//}

namespace pairs_uav_dji_tello_api
{

/* class Api //{ */

class Api : public pairs_uav_hw_api::PairsUavHwApi {

public:
  ~Api(){};

  void initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers);

  void destroy();

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;

  // | --------------------- status methods --------------------- |

  pairs_msgs::msg::HwApiStatus       getStatus();
  pairs_msgs::msg::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(const pairs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg);
  bool callbackControlGroupCmd(const pairs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg);
  bool callbackAttitudeRateCmd(const pairs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg);
  bool callbackAttitudeCmd(const pairs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgRateCmd(const pairs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgCmd(const pairs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgRateCmd(const pairs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgCmd(const pairs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg);
  bool callbackPositionCmd(const pairs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg);

  void callbackTrackerCmd(const pairs_msgs::msg::TrackerCommand::ConstSharedPtr msg);

  // | -------------------- service callbacks ------------------- |

  pairs_lib::Task<std::tuple<bool, std::string>> callbackArming(const bool &request);
  pairs_lib::Task<std::tuple<bool, std::string>> callbackOffboard(void);

private:
  bool is_initialized_ = false;

  std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- subscribers ---------------------- |

  pairs_lib::SubscriberHandler<std_msgs::msg::Bool> sh_armed_;

  pairs_lib::SubscriberHandler<geometry_msgs::msg::PoseStamped>  sh_pose_;
  pairs_lib::SubscriberHandler<geometry_msgs::msg::TwistStamped> sh_twist_;
  pairs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState>   sh_battery_;
  pairs_lib::SubscriberHandler<sensor_msgs::msg::Imu>            sh_imu_;
  pairs_lib::SubscriberHandler<std_msgs::msg::Float64>           sh_height_;

  void callbackArmed(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void callbackPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void callbackTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void callbackBattery(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void callbackHeight(const std_msgs::msg::Float64::ConstSharedPtr msg);

  void publishOdom(void);

  // | --------------------- integrated pose -------------------- |

  Eigen::Vector3d pos_;

  rclcpp::Time last_update_;

  // | ----------------------- publishers ----------------------- |

  pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiVelocityHdgRateCmd> ph_cmd_;

  // | ------------------------ services ------------------------ |

  pairs_lib::ServiceClientHandler<std_srvs::srv::SetBool>    sch_arm_;
  pairs_lib::ServiceClientHandler<mavros_msgs::srv::SetMode> sch_set_mode_;

  // | ------------------------ variables ----------------------- |

  std::atomic<bool> offboard_ = false;
  std::string       mode_;
  std::atomic<bool> armed_     = false;
  std::atomic<bool> connected_ = false;
  std::mutex        mutex_status_;

  geometry_msgs::msg::PoseStamped pose_;
  std::mutex                      mutex_pose_;

  geometry_msgs::msg::TwistStamped twist_;
  std::mutex                       mutex_twist_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void Api::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers) {

  node_  = node;
  clock_ = node_->get_clock();

  common_handlers_ = common_handlers;

  cbkgrp_subs_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_   = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  pos_ << 0, 0, 0;
  last_update_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  // | ------------------- loading parameters ------------------- |

  pairs_lib::ParamLoader local_param_loader(node_, "TelloApi");

  std::string custom_config_path;

  common_handlers_->main_param_loader->loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    local_param_loader.addYamlFile(custom_config_path);
  }

  std::vector<std::string> config_files;
  common_handlers_->main_param_loader->loadParamReusable("configs", config_files);

  if (!common_handlers_->main_param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "loading config file '%s'", config_file.c_str());
    local_param_loader.addYamlFile(config_file);
  }

  // | ----------------------- subscribers ---------------------- |

  pairs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.node_name                           = "PairsHwTelloApi";
  shopts.no_message_timeout                  = pairs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_armed_ = pairs_lib::SubscriberHandler<std_msgs::msg::Bool>(shopts, "~/armed_in", &Api::callbackArmed, this);

  sh_pose_ = pairs_lib::SubscriberHandler<geometry_msgs::msg::PoseStamped>(shopts, "~/pose_in", &Api::callbackPose, this);

  sh_twist_ = pairs_lib::SubscriberHandler<geometry_msgs::msg::TwistStamped>(shopts, "~/twist_in", &Api::callbackTwist, this);

  sh_battery_ = pairs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState>(shopts, "~/battery_in", &Api::callbackBattery, this);

  sh_imu_ = pairs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "~/imu_in", &Api::callbackImu, this);

  sh_height_ = pairs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/height_in", &Api::callbackHeight, this);

  // | --------------------- service clients -------------------- |

  sch_arm_      = pairs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/arm_out", cbkgrp_sc_);
  sch_set_mode_ = pairs_lib::ServiceClientHandler<mavros_msgs::srv::SetMode>(node_, "~/set_mode_out", cbkgrp_sc_);

  // | ----------------------- publishers ----------------------- |

  ph_cmd_ = pairs_lib::PublisherHandler<pairs_msgs::msg::HwApiVelocityHdgRateCmd>(node_, "~/cmd_out");

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;
}

//}

/* getStatus() //{ */

pairs_msgs::msg::HwApiStatus Api::getStatus() {

  pairs_msgs::msg::HwApiStatus status;

  status.stamp = clock_->now();

  {
    std::scoped_lock lock(mutex_status_);

    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  return status;
}

//}

/* getCapabilities() //{ */

pairs_msgs::msg::HwApiCapabilities Api::getCapabilities() {

  pairs_msgs::msg::HwApiCapabilities capabilities;

  capabilities.api_name = "TelloApi";
  capabilities.stamp    = node_->now();

  capabilities.accepts_velocity_hdg_rate_cmd = true;

  capabilities.produces_odometry        = true;
  capabilities.produces_battery_state   = true;
  capabilities.produces_orientation     = true;
  capabilities.produces_distance_sensor = true;
  capabilities.produces_imu             = true;

  return capabilities;
}

//}

/* callbackControlActuatorCmd() //{ */

bool Api::callbackActuatorCmd(const pairs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting actuator cmd");

  return false;
}

//}

/* callbackControlGroupCmd() //{ */

bool Api::callbackControlGroupCmd(const pairs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting control group cmd");

  return true;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool Api::callbackAttitudeRateCmd(const pairs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude rate cmd");

  return true;
}

//}

/* callbackAttitudeCmd() //{ */

bool Api::callbackAttitudeCmd(const pairs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude cmd");

  return false;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool Api::callbackAccelerationHdgRateCmd(const pairs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting acceleration+hdg rate cmd");

  return false;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool Api::callbackAccelerationHdgCmd(const pairs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting acceleration+hdg cmd");

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool Api::callbackVelocityHdgRateCmd(const pairs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting velocity+hdg rate cmd");

  auto pose = sh_pose_.getMsg();

  Eigen::Vector3d vel_world(msg->velocity.x, msg->velocity.y, msg->velocity.z);

  Eigen::Matrix3d R = pairs_lib::AttitudeConverter(pose->pose.orientation);

  Eigen::Vector3d vel_body = R.transpose() * vel_world;

  pairs_msgs::msg::HwApiVelocityHdgRateCmd cmd_out;

  cmd_out            = *msg;
  cmd_out.velocity.x = vel_body[0];
  cmd_out.velocity.y = vel_body[1];
  cmd_out.velocity.z = vel_body[2];

  ph_cmd_.publish(cmd_out);

  return true;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool Api::callbackVelocityHdgCmd(const pairs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting velocity+hdg cmd");

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool Api::callbackPositionCmd(const pairs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting position cmd");

  return false;
}

//}

/* callbackTrackerCmd() //{ */

void Api::callbackTrackerCmd(const pairs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting tracker cmd");
}

//}

/* callbackArming() //{ */

pairs_lib::Task<std::tuple<bool, std::string>> Api::callbackArming([[maybe_unused]] const bool &request) {

  std::stringstream ss;

  auto srv_out = std::make_shared<std_srvs::srv::SetBool::Request>();

  srv_out->data = request;

  if (!request) {
    offboard_ = false;
  }

  RCLCPP_INFO(node_->get_logger(), "calling for %s", request ? "arming" : "disarming");

  auto response = co_await sch_arm_.callAwaitable(srv_out);

  if (response) {

    if (response.value()->success) {
      ss << "service call for " << (request ? "arming" : "disarming") << " was successful";
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());

    } else {
      ss << "service call for " << (request ? "arming" : "disarming") << " failed";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
    }

  } else {
    ss << "calling for " << (request ? "arming" : "disarming") << " resulted in failure: '" << response.value()->message << "'";
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "" << ss.str());
  }

  co_return {response.value()->success, ss.str()};
}

//}

/* callbackOffboard() //{ */

pairs_lib::Task<std::tuple<bool, std::string>> Api::callbackOffboard(void) {

  std::stringstream ss;

  if (!armed_) {

    offboard_ = false;
    ss << "not armed";
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "%s", ss.str().c_str());
    co_return {false, ss.str()};

  } else {

    offboard_ = true;
    ss << "success";
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "%s", ss.str().c_str());
    co_return {true, ss.str()};
  }
}

//}

/* destroy() //{ */

void Api::destroy() {
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackTelloStatus() */

void Api::callbackArmed(const std_msgs::msg::Bool::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting armed state");

  auto state = msg;

  {
    std::scoped_lock lock(mutex_status_);

    armed_     = state->data;
    connected_ = true;
  }

  // | ----------------- publish the diagnostics ---------------- |

  pairs_msgs::msg::HwApiStatus status;

  {
    std::scoped_lock lock(mutex_status_);

    status.stamp     = clock_->now();
    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  common_handlers_->publishers.publishStatus(status);
}

//}

/* calbackPose() //{ */

void Api::callbackPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting local odometry");

  // | --------------- publish the local odometry --------------- |

  pairs_lib::set_mutexed(mutex_pose_, *msg, pose_);
}

//}

/* calbackHeight() //{ */

void Api::callbackHeight(const std_msgs::msg::Float64::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting height");

  sensor_msgs::msg::Range range_out;
  range_out.min_range = 0.1;
  range_out.max_range = 10.0;
  range_out.range     = msg->data;

  common_handlers_->publishers.publishDistanceSensor(range_out);
}

//}

/* calbackTwist() //{ */

void Api::callbackTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting local odometry");

  if (!sh_pose_.hasMsg()) {
    return;
  }

  // | ----------------- integrate the velocity ----------------- |

  if (last_update_.seconds() == 0) {
    last_update_ = clock_->now();
    return;
  }

  double dt    = (clock_->now() - last_update_).seconds();
  last_update_ = clock_->now();

  auto pose  = sh_pose_.getMsg();
  auto twist = msg;

  Eigen::Vector3d vel_world(twist->twist.linear.x, twist->twist.linear.y, twist->twist.linear.z);
  Eigen::Matrix3d R = pairs_lib::AttitudeConverter(pose->pose.orientation);

  /* Eigen::Vector3d vel_world = R * vel_body; */

  pos_ += vel_world * dt;

  // | --------------- publish the local odometry --------------- |

  pairs_lib::set_mutexed(mutex_twist_, *twist, twist_);

  publishOdom();

  clock_->sleep_for(std::chrono::duration<double>(0.01));

  publishOdom();
}

//}

/* calbackImu() //{ */

void Api::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting imu");

  common_handlers_->publishers.publishIMU(*msg);
}

//}

/* callbackBattery() //{ */

void Api::callbackBattery(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting battery");

  common_handlers_->publishers.publishBatteryState(*msg);
}

//}

// | ------------------------ routines ------------------------ |

/* publishOdom() //{ */

void Api::publishOdom(void) {

  // | ---------------- fill in the odom message ---------------- |

  auto twist = pairs_lib::get_mutexed(mutex_twist_, twist_);
  auto pose  = pairs_lib::get_mutexed(mutex_pose_, pose_);

  nav_msgs::msg::Odometry odom;

  odom.header          = pose.header;
  odom.header.frame_id = common_handlers_->getWorldFrameName();
  odom.header.stamp    = clock_->now();
  /* odom.pose.pose.position.x = pos_[0]; */
  /* odom.pose.pose.position.y = pos_[1]; */
  /* odom.pose.pose.position.z = pos_[2]; */

  odom.pose.pose = pose.pose;

  odom.child_frame_id = common_handlers_->getUavName() + "/" + common_handlers_->getBodyFrameName();
  odom.twist.twist    = twist.twist;

  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;

  common_handlers_->publishers.publishOdometry(odom);

  // | ----------------- publish the orientation ---------------- |

  geometry_msgs::msg::QuaternionStamped quat;

  quat.header     = pose.header;
  quat.quaternion = pose.pose.orientation;

  common_handlers_->publishers.publishOrientation(quat);

  // | ---------------- publish angular velocity ---------------- |

  /* geometry_msgs::Vector3Stamped angular_velocity; */

  /* angular_velocity.header.stamp    = pose.header.stamp; */
  /* angular_velocity.header.frame_id = common_handlers_->getUavName() + "/" + common_handlers_->getBodyFrameName(); */

  /* angular_velocity.vector.x = twist.twist.angular.x; */
  /* angular_velocity.vector.y = twist.twist.angular.y; */
  /* angular_velocity.vector.z = twist.twist.angular.z; */

  /* common_handlers_->publishers.publishAngularVelocity(angular_velocity); */

  // | ------------------ publish emulated gnss ----------------- |

  /* { */
  /*   double lat; */
  /*   double lon; */

  /*   pairs_lib::UTMtoLL(pose.pose.position.y + _utm_y_, pose.pose.position.x + _utm_x_, "32T", lat, lon); */

  /*   sensor_msgs::NavSatFix gnss; */

  /*   gnss.header.stamp = pose.header.stamp; */

  /*   gnss.latitude  = lat; */
  /*   gnss.longitude = lon; */
  /*   gnss.altitude  = pose.pose.position.z + _amsl_; */

  /*   common_handlers_->publishers.publishGNSS(gnss); */
  /* } */

  // | -------------------- publish altitude -------------------- |


  /* pairs_msgs::HwApiAltitude altitude; */

  /* altitude.stamp = pose.header.stamp; */

  /* altitude.amsl = pose.pose.position.z + _amsl_; */

  /* common_handlers_->publishers.publishAltitude(altitude); */

  // | --------------------- publish heading -------------------- |

  /* double heading = pairs_lib::AttitudeConverter(pose.pose.orientation).getHeading(); */

  /* pairs_msgs::Float64Stamped hdg; */

  /* hdg.header.stamp = ros::Time::now(); */
  /* hdg.value        = heading; */

  /* common_handlers_->publishers.publishMagnetometerHeading(hdg); */
}

//}

}  // namespace pairs_uav_dji_tello_api

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pairs_uav_dji_tello_api::Api, pairs_uav_hw_api::PairsUavHwApi)
