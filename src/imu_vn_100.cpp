/*
 * Copyright [2015] [Ke Sun]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <imu_vn_100/imu_vn_100.h>

namespace imu_vn_100 {

// LESS HACK IS STILL HACK
ImuVn100 *imu_vn_100_ptr;

using sensor_msgs::Imu;
using sensor_msgs::MagneticField;
using sensor_msgs::FluidPressure;
using sensor_msgs::Temperature;

void RosVector3FromVnVector3(geometry_msgs::Vector3 &ros_vec3,
                             const VnVector3 &vn_vec3);
void RosQuaternionFromVnQuaternion(geometry_msgs::Quaternion &ros_quat,
                                   const VnQuaternion &vn_quat);
void FillImuMessage(sensor_msgs::Imu &imu_msg,
                    const VnDeviceCompositeData &data, bool binary_output);

void AsyncListener(void *sender, VnDeviceCompositeData *data) {
  imu_vn_100_ptr->PublishData(*data);
}

void VnErrorHandler(void *sender, VN_ERROR_CODE error_code) {
  imu_vn_100_ptr->ErrorHandler(error_code);
}

constexpr int ImuVn100::kBaseImuRate;
constexpr int ImuVn100::kDefaultImuRate;
constexpr int ImuVn100::kDefaultSyncOutRate;

void ImuVn100::SyncInfo::Update(const unsigned sync_count,
                                const ros::Time &sync_time) {
  if (rate <= 0)
    return;

  if (count != sync_count) {
    count = sync_count;
    time = sync_time;
  }
}

bool ImuVn100::SyncInfo::SyncEnabled() const { return rate > 0; }

void ImuVn100::SyncInfo::FixSyncRate() {
  // Check the sync out rate
  if (SyncEnabled()) {
    if (ImuVn100::kBaseImuRate % rate != 0) {
      rate = ImuVn100::kBaseImuRate / (ImuVn100::kBaseImuRate / rate);
      ROS_INFO("Set SYNC_OUT_RATE to %d", rate);
    }
    skip_count =
        (std::floor(ImuVn100::kBaseImuRate / static_cast<double>(rate) +
                    0.5f)) -
        1;

    if (pulse_width_us > 10000) {
      ROS_INFO("Sync out pulse with is over 10ms. Reset to 1ms");
      pulse_width_us = 1000;
    }
    rate_double = rate;
  }

  ROS_INFO("Sync out rate: %d", rate);
}

ImuVn100::ImuVn100(const ros::NodeHandle &pnh)
    : pnh_(pnh), port_(std::string("/dev/ttyUSB0")), baudrate_(921600),
      frame_id_(std::string("imu")) {

  std::string xTopic = "xOrientation";
  std::string yTopic = "yOrientatoin";
  std::string zTopic = "zOrientation";
  xOrientationPub_ = pnh_.advertise<std_msgs::Float64>(xTopic, 1);
  yOrientationPub_ = pnh_.advertise<std_msgs::Float64>(yTopic, 1);
  zOrientationPub_ = pnh_.advertise<std_msgs::Float64>(zTopic, 1);
  zVelPub_ = pnh_.advertise<std_msgs::Float64>("zVel", 1);

  Initialize();
  imu_vn_100_ptr = this;
}

ImuVn100::~ImuVn100() { Disconnect(); }

void ImuVn100::FixImuRate() {
  if (imu_rate_ <= 0) {
    ROS_WARN("Imu rate %d is < 0. Set to %d", imu_rate_, kDefaultImuRate);
    imu_rate_ = kDefaultImuRate;
  }

  if (kBaseImuRate % imu_rate_ != 0) {
    int imu_rate_old = imu_rate_;
    imu_rate_ = kBaseImuRate / (kBaseImuRate / imu_rate_old);
    ROS_WARN("Imu rate %d cannot evenly decimate base rate %d, reset to %d",
             imu_rate_old, kBaseImuRate, imu_rate_);
  }
}

void ImuVn100::LoadParameters() {
  pnh_.param<std::string>("port", port_, std::string("/dev/ttyUSB0"));
  pnh_.param<std::string>("frame_id", frame_id_, pnh_.getNamespace());
  pnh_.param("baudrate", baudrate_, 115200);
  pnh_.param("imu_rate", imu_rate_, kDefaultImuRate);

  pnh_.param("enable_mag", enable_mag_, true);
  pnh_.param("enable_pres", enable_pres_, true);
  pnh_.param("enable_temp", enable_temp_, true);
  pnh_.param("enable_frame_rotation", enable_frame_rotation_, false);

  pnh_.param("frame_rotation_matrix", frame_rotation_);

  pnh_.param("sync_rate", sync_info_.rate, kDefaultSyncOutRate);
  pnh_.param("sync_pulse_width_us", sync_info_.pulse_width_us, 1000);

  pnh_.param("binary_output", binary_output_, true);

  FixImuRate();
  sync_info_.FixSyncRate();
}

void ImuVn100::CreateDiagnosedPublishers() {
  imu_rate_double_ = imu_rate_;
  pd_imu_.Create<Imu>(pnh_, "imu", updater_, imu_rate_double_);
  pd_twist_.Create<geometry_msgs::TwistStamped>(pnh_, "twist", updater_,
                                                imu_rate_double_);

  srv_tare_ = pnh_.advertiseService("tare", &ImuVn100::TareOrientation, this);

  if (enable_mag_) {
    pd_mag_.Create<MagneticField>(pnh_, "magnetic_field", updater_,
                                  imu_rate_double_);
  }
  if (enable_pres_) {
    pd_pres_.Create<FluidPressure>(pnh_, "fluid_pressure", updater_,
                                   imu_rate_double_);
  }
  if (enable_temp_) {
    pd_temp_.Create<Temperature>(pnh_, "temperature", updater_,
                                 imu_rate_double_);
  }
}

void ImuVn100::Initialize() {
  LoadParameters();

  for (auto i : frame_rotation_) {
    ROS_INFO("%f", i);
  }

  ROS_INFO("%d", frame_rotation_.size());
  ROS_INFO("%d", enable_frame_rotation_);

  ROS_DEBUG("Connecting to device");
  // Only function where error handler is needed because you can't registerer
  // the error handler until the device has been connected
  ErrorHandler(vn100_connect(&imu_, port_.c_str(), 115200));
  ros::Duration(0.5).sleep();
  ROS_INFO("Connected to device at %s", port_.c_str());

  VnErrorCode error_code =
      vn100_registerErrorCodeReceivedListener(&imu_, &VnErrorHandler);
  if (error_code != VNERR_NO_ERROR) {
    throw std::runtime_error("Error attempting to register error handler");
  }
  ROS_INFO("Error handler successfully registered");

  unsigned int old_baudrate;
  vn100_getSerialBaudRate(&imu_, &old_baudrate);
  ROS_INFO("Default serial baudrate: %u", old_baudrate);

  ROS_INFO("Set serial baudrate to %d", baudrate_);
  vn100_setSerialBaudRate(&imu_, baudrate_, true);

  ROS_DEBUG("Disconnecting the device");
  vn100_disconnect(&imu_);
  ros::Duration(0.5).sleep();

  ROS_DEBUG("Reconnecting to device");
  vn100_connect(&imu_, port_.c_str(), baudrate_);
  ros::Duration(0.5).sleep();
  ROS_INFO("Connected to device at %s", port_.c_str());

  vn100_getSerialBaudRate(&imu_, &old_baudrate);
  ROS_INFO("New serial baudrate: %u", old_baudrate);

  // Idle the device for intialization
  vn100_pauseAsyncOutputs(&imu_, true);

  // Pitch counterclockwise rotation of 90 degrees
  // 0 0 1
  // 0 1 0
  //-1 0 1

  ROS_INFO("HERE");

  // Set a new reference frame and reset the imu
  VnMatrix3x3 refFrame;

  refFrame.c00 = 0, refFrame.c01 = 0, refFrame.c02 = -1;
  refFrame.c10 = 1, refFrame.c11 = 0, refFrame.c12 = 0;
  refFrame.c20 = 0, refFrame.c21 = -1, refFrame.c22 = 0;

  //refFrame.c00 = 1, refFrame.c01 = 0, refFrame.c02 = 0;
  //refFrame.c10 = 0, refFrame.c11 = 1, refFrame.c12 = 0;
  //refFrame.c20 = 0, refFrame.c21 = 0, refFrame.c22 = 1;

  vn100_setReferenceFrameRotation(&imu_, refFrame, true);

  vn100_writeSettings(&imu_, true);

  vn100_reset(&imu_);

  ros::Duration(1).sleep();

  ROS_INFO("Fetching device info.");
  char model_number_buffer[30] = {0};
  int hardware_revision = 0;
  char serial_number_buffer[30] = {0};
  char firmware_version_buffer[30] = {0};

  vn100_getModelNumber(&imu_, model_number_buffer, 30);
  ROS_INFO("Model number: %s", model_number_buffer);
  vn100_getHardwareRevision(&imu_, &hardware_revision);
  ROS_INFO("Hardware revision: %d", hardware_revision);
  vn100_getSerialNumber(&imu_, serial_number_buffer, 30);
  ROS_INFO("Serial number: %s", serial_number_buffer);
  vn100_getFirmwareVersion(&imu_, firmware_version_buffer, 30);
  ROS_INFO("Firmware version: %s", firmware_version_buffer);

  if (sync_info_.SyncEnabled()) {
    ROS_INFO("Set Synchronization Control Register (id:32).");
    vn100_setSynchronizationControl(
        &imu_, SYNCINMODE_COUNT, SYNCINEDGE_RISING, 0, SYNCOUTMODE_IMU_START,
        SYNCOUTPOLARITY_POSITIVE, sync_info_.skip_count,
        sync_info_.pulse_width_us * 1000, true);

    if (!binary_output_) {
      ROS_INFO("Set Communication Protocal Control Register (id:30).");
      vn100_setCommunicationProtocolControl(
          &imu_, SERIALCOUNT_SYNCOUT_COUNT, SERIALSTATUS_OFF, SPICOUNT_NONE,
          SPISTATUS_OFF, SERIALCHECKSUM_8BIT, SPICHECKSUM_8BIT, ERRORMODE_SEND,
          true);
    }
  }

  CreateDiagnosedPublishers();

  auto hardware_id = std::string("vn100-") + std::string(model_number_buffer) +
                     std::string(serial_number_buffer);
  updater_.setHardwareID(hardware_id);
}

void ImuVn100::Stream(bool async) {
  // Pause the device first
  vn100_pauseAsyncOutputs(&imu_, true);

  VnMatrix3x3 m;
  vn100_getReferenceFrameRotation(&imu_, &m);
  ROS_INFO("%.2f  | %.2f  | %.2f", m.c00, m.c01, m.c02);
  ROS_INFO("%.2f  | %.2f  | %.2f", m.c10, m.c11, m.c12);
  ROS_INFO("%.2f  | %.2f  | %.2f", m.c20, m.c21, m.c22);

  // Pitch counterclockwise rotation of 90 degrees
  // 0 0 1
  // 0 1 0
  //-1 0 1

  if (async) {
    vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true);

    if (binary_output_) {
      // Set the binary output data type and data rate
      vn100_setBinaryOutput1Configuration(
          &imu_, BINARY_ASYNC_MODE_SERIAL_1, kBaseImuRate / imu_rate_,
          BG1_ANGULAR_RATE | BG1_YPR,
          // BG1_IMU,
          BG3_TEMP | BG3_PRES, BG5_ACCEL_NED, true);
    } else {
      // Set the ASCII output data type and data rate
      // ROS_INFO("Configure the output data type and frequency (id: 6 & 7)");
      vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_VNIMU, true);
    }

    // Add a callback function for new data event
    vn100_registerAsyncDataReceivedListener(&imu_, &AsyncListener);

    ROS_INFO("Setting IMU rate to %d", imu_rate_);
    vn100_setAsynchronousDataOutputFrequency(&imu_, imu_rate_, true);
  } else {
    // Mute the stream
    ROS_DEBUG("Mute the device");
    vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true);
    // Remove the callback function for new data event
    vn100_unregisterAsyncDataReceivedListener(&imu_, &AsyncListener);
  }

  // Resume the device
  vn100_resumeAsyncOutputs(&imu_, true);
}

VN_ERROR_CODE ImuVn100::Resume(bool need_reply) {
  return vn100_resumeAsyncOutputs(&imu_, need_reply);
}

VN_ERROR_CODE ImuVn100::Idle(bool need_reply) {
  return vn100_pauseAsyncOutputs(&imu_, need_reply);
}

VN_ERROR_CODE ImuVn100::ZeroOrientation(bool need_reply) {
  VnErrorCode error_code;

  // Idle the Imu, wait for a reply
  if ((error_code = Idle(need_reply)) != VNERR_NO_ERROR)
    return error_code;

  if ((error_code = vn100_tare(&imu_, need_reply)) != VNERR_NO_ERROR)
    return error_code;

  if ((error_code = Resume(need_reply)) != VNERR_NO_ERROR)
    return error_code;

  return VNERR_NO_ERROR;
}

void ImuVn100::Reset() { vn100_reset(&imu_); }

void ImuVn100::Disconnect() {
  // TODO: why reset the device?
  Reset();
  vn100_disconnect(&imu_);
}

void ImuVn100::PublishData(const VnDeviceCompositeData &data) {
  sensor_msgs::Imu imu_msg;
  geometry_msgs::TwistStamped twist_msg;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = frame_id_;

  twist_msg.header = imu_msg.header;
  RosVector3FromVnVector3(twist_msg.twist.angular, data.angularRate);
  RosVector3FromVnVector3(twist_msg.twist.linear, data.acceleration);

  FillImuMessage(imu_msg, data, binary_output_);
  pd_imu_.Publish(imu_msg);
  pd_twist_.Publish(twist_msg);
  std_msgs::Float64 msg;
  msg.data = imu_msg.orientation.x;
  xOrientationPub_.publish(msg);
  msg.data = imu_msg.orientation.y;
  yOrientationPub_.publish(msg);
  msg.data = imu_msg.orientation.z;
  zOrientationPub_.publish(msg);
  msg.data = imu_msg.angular_velocity.z;
  zVelPub_.publish(msg);
  if (enable_mag_) {
    sensor_msgs::MagneticField mag_msg;
    mag_msg.header = imu_msg.header;
    RosVector3FromVnVector3(mag_msg.magnetic_field, data.magnetic);
    pd_mag_.Publish(mag_msg);
  }

  if (enable_pres_) {
    sensor_msgs::FluidPressure pres_msg;
    pres_msg.header = imu_msg.header;
    pres_msg.fluid_pressure = data.pressure;
    pd_pres_.Publish(pres_msg);
  }

  if (enable_temp_) {
    sensor_msgs::Temperature temp_msg;
    temp_msg.header = imu_msg.header;
    temp_msg.temperature = data.temperature;
    pd_temp_.Publish(temp_msg);
  }

  sync_info_.Update(data.syncInCnt, imu_msg.header.stamp);

  updater_.update();
}

void ImuVn100::ErrorHandler(const VnErrorCode &error_code) {
  VnEnsure(error_code);
}

bool ImuVn100::TareOrientation(imu_vn_100::Tare::Request &req,
                               imu_vn_100::Tare::Response &res) {
  if (ZeroOrientation(true) == VNERR_NO_ERROR) {
    res.status = 0;
    return true;
  } else {
    res.status = -1;
    return false;
  }
}

void VnEnsure(const VnErrorCode &error_code) {
  if (error_code == VNERR_NO_ERROR)
    return;

  switch (error_code) {
  case VNERR_UNKNOWN_ERROR:
    throw std::runtime_error("VN: Unknown error");
  case VNERR_NOT_IMPLEMENTED:
    throw std::runtime_error("VN: Not implemented");
  case VNERR_TIMEOUT:
    ROS_WARN("Opertation time out");
    break;
  case VNERR_SENSOR_INVALID_PARAMETER:
    ROS_WARN("VN: Sensor invalid paramter");
    break;
  case VNERR_INVALID_VALUE:
    ROS_WARN("VN: Invalid value");
    break;
  case VNERR_FILE_NOT_FOUND:
    throw std::runtime_error("VN: File not found");
    break;
  case VNERR_NOT_CONNECTED:
    throw std::runtime_error("VN: not connected");
  case VNERR_PERMISSION_DENIED:
    throw std::runtime_error("VN: Permission denied");
  default:
    ROS_WARN("Unhandled error type");
  }
}

void RosVector3FromVnVector3(geometry_msgs::Vector3 &ros_vec3,
                             const VnVector3 &vn_vec3) {
  ros_vec3.x = vn_vec3.c0;
  ros_vec3.y = vn_vec3.c1;
  ros_vec3.z = vn_vec3.c2;
}

void RosVector3FromVnYpr(geometry_msgs::Vector3 &ros_vec3,
                         const VnYpr &vn_ypr) {
  ros_vec3.x = vn_ypr.yaw;
  ros_vec3.y = vn_ypr.pitch;
  ros_vec3.z = vn_ypr.roll;
}

void RosQuaternionFromVnQuaternion(geometry_msgs::Quaternion &ros_quat,
                                   const VnQuaternion &vn_quat) {
  ros_quat.x = vn_quat.x;
  ros_quat.y = vn_quat.y;
  ros_quat.z = vn_quat.z;
  ros_quat.w = vn_quat.w;
}

void RosQuaternionFromVnYpr(geometry_msgs::Quaternion &ros_quat,
                            const VnYpr &vn_ypr) {
  double roll_rads = vn_ypr.roll * M_PI / 180;
  double pitch_rads = vn_ypr.pitch * M_PI / 180;
  double yaw_rads = vn_ypr.yaw * M_PI / 180;

  tf2::Quaternion rotation;
  rotation.setRPY(roll_rads, pitch_rads, yaw_rads);
  rotation.normalize();
  ros_quat = tf2::toMsg(rotation);
}

void FillImuMessage(sensor_msgs::Imu &imu_msg,
                    const VnDeviceCompositeData &data, bool binary_output) {
  if (binary_output) {
    RosQuaternionFromVnYpr(imu_msg.orientation, data.ypr);
    // NOTE: The IMU angular velocity and linear acceleration outputs are
    // swapped. And also why are they different?
    RosVector3FromVnVector3(imu_msg.angular_velocity, data.angularRate);
    // RosVector3FromVnVector3(imu_msg.linear_acceleration,
    // data.linearAccelBody);
  } else {
    RosQuaternionFromVnQuaternion(imu_msg.orientation, data.quaternion);

    RosVector3FromVnVector3(imu_msg.angular_velocity, data.angularRate);
    RosVector3FromVnVector3(imu_msg.linear_acceleration, data.acceleration);
  }
}

} //  namespace imu_vn_100
