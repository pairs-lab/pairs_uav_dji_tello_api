/* includes //{ */

#include <ros/ros.h>

#include <pairs_uav_hw_api/api.h>

#include <pairs_lib/param_loader.h>
#include <pairs_lib/attitude_converter.h>
#include <pairs_lib/mutex.h>
#include <pairs_lib/publisher_handler.h>
#include <pairs_lib/subscribe_handler.h>
#include <pairs_lib/service_client_handler.h>

#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

//}

namespace pairs_uav_dji_tello_api
{

/* class PairsUavDjiTelloApi //{ */

class PairsUavDjiTelloApi : public pairs_uav_hw_api::PairsUavHwApi {

public:
  ~PairsUavDjiTelloApi(){};

  void initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers);

  // | --------------------- status methods --------------------- |

  pairs_msgs::HwApiStatus       getStatus();
  pairs_msgs::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(const pairs_msgs::HwApiActuatorCmd::ConstPtr msg);
  bool callbackControlGroupCmd(const pairs_msgs::HwApiControlGroupCmd::ConstPtr msg);
  bool callbackAttitudeRateCmd(const pairs_msgs::HwApiAttitudeRateCmd::ConstPtr msg);
  bool callbackAttitudeCmd(const pairs_msgs::HwApiAttitudeCmd::ConstPtr msg);
  bool callbackAccelerationHdgRateCmd(const pairs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg);
  bool callbackAccelerationHdgCmd(const pairs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg);
  bool callbackVelocityHdgRateCmd(const pairs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg);
  bool callbackVelocityHdgCmd(const pairs_msgs::HwApiVelocityHdgCmd::ConstPtr msg);
  bool callbackPositionCmd(const pairs_msgs::HwApiPositionCmd::ConstPtr msg);

  void callbackTrackerCmd(const pairs_msgs::TrackerCommand::ConstPtr msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool &request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  bool is_initialized_ = false;

  std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- subscribers ---------------------- |

  pairs_lib::SubscribeHandler<std_msgs::Bool> sh_armed_;

  void callbackArmed(const std_msgs::Bool::ConstPtr msg);

  pairs_lib::SubscribeHandler<geometry_msgs::PoseStamped> sh_pose_;
  void                                                  callbackPose(const geometry_msgs::PoseStamped::ConstPtr msg);

  pairs_lib::SubscribeHandler<geometry_msgs::TwistStamped> sh_twist_;
  void                                                   callbackTwist(const geometry_msgs::TwistStamped::ConstPtr msg);

  pairs_lib::SubscribeHandler<sensor_msgs::BatteryState> sh_battery_;
  void                                                 callbackBattery(const sensor_msgs::BatteryState::ConstPtr msg);

  pairs_lib::SubscribeHandler<sensor_msgs::Imu> sh_imu_;
  void                                        callbackImu(const sensor_msgs::Imu::ConstPtr msg);

  pairs_lib::SubscribeHandler<std_msgs::Float64> sh_height_;
  void                                         callbackHeight(const std_msgs::Float64::ConstPtr msg);


  void publishOdom(void);

  // | --------------------- integrated pose -------------------- |

  Eigen::Vector3d pos_;

  ros::Time last_update_;

  // | ----------------------- publishers ----------------------- |

  pairs_lib::PublisherHandler<pairs_msgs::HwApiVelocityHdgRateCmd> ph_cmd_;

  // | ------------------------ services ------------------------ |

  pairs_lib::ServiceClientHandler<std_srvs::SetBool>    sch_arm_;
  pairs_lib::ServiceClientHandler<mavros_msgs::SetMode> sch_set_mode_;

  // | ------------------------ variables ----------------------- |

  std::atomic<bool> offboard_ = false;
  std::string       mode_;
  std::atomic<bool> armed_     = false;
  std::atomic<bool> connected_ = false;
  std::mutex        mutex_status_;

  geometry_msgs::PoseStamped pose_;
  std::mutex                 mutex_pose_;

  geometry_msgs::TwistStamped twist_;
  std::mutex                  mutex_twist_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void PairsUavDjiTelloApi::initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<pairs_uav_hw_api::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh);

  common_handlers_ = common_handlers;

  pos_ << 0, 0, 0;
  last_update_ = ros::Time(0);

  // | ------------------- loading parameters ------------------- |

  pairs_lib::ParamLoader param_loader(nh_, "PairsUavHwApi");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[PairsUavDjiTelloApi]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  pairs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "PairsHwTelloApi";
  shopts.no_message_timeout = pairs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_armed_ = pairs_lib::SubscribeHandler<std_msgs::Bool>(shopts, "armed_in", &PairsUavDjiTelloApi::callbackArmed, this);

  sh_pose_ = pairs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, "pose_in", &PairsUavDjiTelloApi::callbackPose, this);

  sh_twist_ = pairs_lib::SubscribeHandler<geometry_msgs::TwistStamped>(shopts, "twist_in", &PairsUavDjiTelloApi::callbackTwist, this);

  sh_battery_ = pairs_lib::SubscribeHandler<sensor_msgs::BatteryState>(shopts, "battery_in", &PairsUavDjiTelloApi::callbackBattery, this);

  sh_imu_ = pairs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, "imu_in", &PairsUavDjiTelloApi::callbackImu, this);

  sh_height_ = pairs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "height_in", &PairsUavDjiTelloApi::callbackHeight, this);

  // | --------------------- service clients -------------------- |

  sch_arm_      = pairs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "arm_out");
  sch_set_mode_ = pairs_lib::ServiceClientHandler<mavros_msgs::SetMode>(nh_, "set_mode_out");

  // | ----------------------- publishers ----------------------- |

  ph_cmd_ = pairs_lib::PublisherHandler<pairs_msgs::HwApiVelocityHdgRateCmd>(nh_, "cmd_out", 1);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[PairsUavDjiTelloApi]: initialized");

  is_initialized_ = true;
}

//}

/* getStatus() //{ */

pairs_msgs::HwApiStatus PairsUavDjiTelloApi::getStatus() {

  pairs_msgs::HwApiStatus status;

  status.stamp = ros::Time::now();

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

pairs_msgs::HwApiCapabilities PairsUavDjiTelloApi::getCapabilities() {

  pairs_msgs::HwApiCapabilities capabilities;

  capabilities.api_name = "TelloApi";
  capabilities.stamp    = ros::Time::now();

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

bool PairsUavDjiTelloApi::callbackActuatorCmd(const pairs_msgs::HwApiActuatorCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting actuator cmd");

  return false;
}

//}

/* callbackControlGroupCmd() //{ */

bool PairsUavDjiTelloApi::callbackControlGroupCmd(const pairs_msgs::HwApiControlGroupCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting control group cmd");

  return true;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool PairsUavDjiTelloApi::callbackAttitudeRateCmd(const pairs_msgs::HwApiAttitudeRateCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting attitude rate cmd");

  return true;
}

//}

/* callbackAttitudeCmd() //{ */

bool PairsUavDjiTelloApi::callbackAttitudeCmd(const pairs_msgs::HwApiAttitudeCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting attitude cmd");

  return false;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool PairsUavDjiTelloApi::callbackAccelerationHdgRateCmd(const pairs_msgs::HwApiAccelerationHdgRateCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting acceleration+hdg rate cmd");

  return false;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool PairsUavDjiTelloApi::callbackAccelerationHdgCmd(const pairs_msgs::HwApiAccelerationHdgCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting acceleration+hdg cmd");

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool PairsUavDjiTelloApi::callbackVelocityHdgRateCmd(const pairs_msgs::HwApiVelocityHdgRateCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting velocity+hdg rate cmd");

  auto pose = sh_pose_.getMsg();

  Eigen::Vector3d vel_world(msg->velocity.x, msg->velocity.y, msg->velocity.z);

  Eigen::Matrix3d R = pairs_lib::AttitudeConverter(pose->pose.orientation);

  Eigen::Vector3d vel_body = R.transpose() * vel_world;

  pairs_msgs::HwApiVelocityHdgRateCmd cmd_out;

  cmd_out            = *msg;
  cmd_out.velocity.x = vel_body[0];
  cmd_out.velocity.y = vel_body[1];
  cmd_out.velocity.z = vel_body[2];

  ph_cmd_.publish(cmd_out);

  return true;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool PairsUavDjiTelloApi::callbackVelocityHdgCmd(const pairs_msgs::HwApiVelocityHdgCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting velocity+hdg cmd");

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool PairsUavDjiTelloApi::callbackPositionCmd(const pairs_msgs::HwApiPositionCmd::ConstPtr msg) {

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting position cmd");

  return false;
}

//}

/* callbackTrackerCmd() //{ */

void PairsUavDjiTelloApi::callbackTrackerCmd(const pairs_msgs::TrackerCommand::ConstPtr msg) {

  ROS_INFO_ONCE("[Api]: getting tracker cmd");
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> PairsUavDjiTelloApi::callbackArming([[maybe_unused]] const bool &request) {

  std::stringstream ss;

  std_srvs::SetBool srv_out;

  srv_out.request.data = request;

  if (!request) {
    offboard_ = false;
  }

  ROS_INFO("[TelloApi]: calling for %s", request ? "arming" : "disarming");

  if (sch_arm_.call(srv_out)) {

    if (srv_out.response.success) {
      ss << "service call for " << (request ? "arming" : "disarming") << " was successful";
      ROS_INFO_STREAM_THROTTLE(1.0, "[TelloApi]: " << ss.str());

    } else {
      ss << "service call for " << (request ? "arming" : "disarming") << " failed";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[TelloApi]: " << ss.str());
    }

  } else {
    ss << "calling for " << (request ? "arming" : "disarming") << " resulted in failure: '" << srv_out.response.message << "'";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TelloApi]: " << ss.str());
  }

  return {srv_out.response.success, ss.str()};
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> PairsUavDjiTelloApi::callbackOffboard(void) {

  std::stringstream ss;

  if (!armed_) {

    offboard_ = false;
    ss << "not armed";
    ROS_ERROR_THROTTLE(1.0, "[TelloApi]: %s", ss.str().c_str());
    return {false, ss.str()};

  } else {

    offboard_ = true;
    ss << "success";
    ROS_ERROR_THROTTLE(1.0, "[TelloApi]: %s", ss.str().c_str());
    return {true, ss.str()};
  }
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackTelloStatus() */

void PairsUavDjiTelloApi::callbackArmed(const std_msgs::Bool::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting armed state");

  auto state = msg;

  {
    std::scoped_lock lock(mutex_status_);

    armed_     = state->data;
    connected_ = true;
  }

  // | ----------------- publish the diagnostics ---------------- |

  pairs_msgs::HwApiStatus status;

  {
    std::scoped_lock lock(mutex_status_);

    status.stamp     = ros::Time::now();
    status.armed     = armed_;
    status.offboard  = offboard_;
    status.connected = connected_;
    status.mode      = mode_;
  }

  common_handlers_->publishers.publishStatus(status);
}

//}

/* calbackPose() //{ */

void PairsUavDjiTelloApi::callbackPose(const geometry_msgs::PoseStamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting local odometry");

  // | --------------- publish the local odometry --------------- |

  pairs_lib::set_mutexed(mutex_pose_, *msg, pose_);
}

//}

/* calbackHeight() //{ */

void PairsUavDjiTelloApi::callbackHeight(const std_msgs::Float64::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting height");

  sensor_msgs::Range range_out;
  range_out.min_range = 0.1;
  range_out.max_range = 10.0;
  range_out.range     = msg->data;

  common_handlers_->publishers.publishDistanceSensor(range_out);
}

//}

/* calbackTwist() //{ */

void PairsUavDjiTelloApi::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting local odometry");

  if (!sh_pose_.hasMsg()) {
    return;
  }

  // | ----------------- integrate the velocity ----------------- |

  if (last_update_ == ros::Time(0)) {
    last_update_ = ros::Time::now();
    return;
  }

  double dt    = (ros::Time::now() - last_update_).toSec();
  last_update_ = ros::Time::now();

  auto pose  = sh_pose_.getMsg();
  auto twist = msg;

  Eigen::Vector3d vel_world(twist->twist.linear.x, twist->twist.linear.y, twist->twist.linear.z);
  Eigen::Matrix3d R = pairs_lib::AttitudeConverter(pose->pose.orientation);

  /* Eigen::Vector3d vel_world = R * vel_body; */

  pos_ += vel_world * dt;

  // | --------------- publish the local odometry --------------- |

  pairs_lib::set_mutexed(mutex_twist_, *twist, twist_);

  publishOdom();
}

//}

/* calbackImu() //{ */

void PairsUavDjiTelloApi::callbackImu(const sensor_msgs::Imu::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting imu");

  common_handlers_->publishers.publishIMU(*msg);
}

//}

/* callbackBattery() //{ */

void PairsUavDjiTelloApi::callbackBattery(const sensor_msgs::BatteryState::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[PairsUavDjiTelloApi]: getting battery");

  common_handlers_->publishers.publishBatteryState(*msg);
}

//}

// | ------------------------ routines ------------------------ |

/* publishOdom() //{ */

void PairsUavDjiTelloApi::publishOdom(void) {

  // | ---------------- fill in the odom message ---------------- |

  auto twist = pairs_lib::get_mutexed(mutex_twist_, twist_);
  auto pose  = pairs_lib::get_mutexed(mutex_pose_, pose_);

  nav_msgs::Odometry odom;

  odom.header          = pose.header;
  odom.header.frame_id = common_handlers_->getWorldFrameName();
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

  geometry_msgs::QuaternionStamped quat;

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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pairs_uav_dji_tello_api::PairsUavDjiTelloApi, pairs_uav_hw_api::PairsUavHwApi)
