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

#ifndef IMU_VN_100_ROS_H_
#define IMU_VN_100_ROS_H_

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Float64.h>

#include <math.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TwistStamped.h>

#include <imu_vn_100/Tare.h>

#include <vn100.h>

namespace imu_vn_100 {

namespace du = diagnostic_updater;
using TopicDiagnosticPtr = boost::shared_ptr<du::TopicDiagnostic>;

// NOTE: there is a DiagnosedPublisher inside diagnostic_updater
// but it does not have a default constructor thus we use this simple one
// instead, which has the same functionality
struct DiagnosedPublisher {
  ros::Publisher pub;
  TopicDiagnosticPtr diag;

  template <typename MessageT>
  void Create(ros::NodeHandle &pnh, const std::string &topic,
              du::Updater &updater, double &rate) {
    pub = pnh.advertise<MessageT>(topic, 1);
    du::FrequencyStatusParam freq_param(&rate, &rate, 0.01, 10);
    du::TimeStampStatusParam time_param(0, 0.5 / rate);
    diag = boost::make_shared<du::TopicDiagnostic>(topic, updater, freq_param,
                                                   time_param);
  }

  template <typename MessageT> void Publish(const MessageT &message) {
    diag->tick(message.header.stamp);
    pub.publish(message);
  }
};

/**
 * @brief ImuVn100 The class is a ros wrapper for the Imu class
 * @author Ke Sun
 */
class ImuVn100 {
public:
  static constexpr int kBaseImuRate = 800;
  static constexpr int kDefaultImuRate = 100;
  static constexpr int kDefaultSyncOutRate = 20;

  explicit ImuVn100(const ros::NodeHandle &pnh);
  ImuVn100(const ImuVn100 &) = delete;
  ImuVn100 &operator=(const ImuVn100 &) = delete;
  ~ImuVn100();

  void Initialize();

  void Stream(bool async = true);

  void PublishData(const VnDeviceCompositeData &data);

  void RequestOnce();

  VN_ERROR_CODE Idle(bool need_reply = true);

  VN_ERROR_CODE Resume(bool need_reply = true);

  VN_ERROR_CODE ZeroOrientation(bool need_reply = true);

  void Reset();

  void Disconnect();

  void Configure();

  void ErrorHandler(const VN_ERROR_CODE &error_code);

  bool TareOrientation(imu_vn_100::Tare::Request &req,
                       imu_vn_100::Tare::Response &res);

  struct SyncInfo {
    unsigned count = 0;
    ros::Time time;

    int rate = -1;
    double rate_double = -1;
    int pulse_width_us = 1000;
    int skip_count = 0;

    void Update(const unsigned sync_count, const ros::Time &sync_time);
    void FixSyncRate();
    bool SyncEnabled() const;
  };

  const SyncInfo sync_info() const { return sync_info_; }

private:
  ros::NodeHandle pnh_;
  Vn100 imu_;

  // Settings
  std::string port_;
  int baudrate_ = 921600;
  int imu_rate_ = kDefaultImuRate;
  double imu_rate_double_ = kDefaultImuRate;
  std::string frame_id_;
  std::vector<double> frame_rotation_;

  bool enable_mag_ = true;
  bool enable_pres_ = true;
  bool enable_temp_ = true;
  bool enable_frame_rotation_ = false;
  bool binary_output_ = true;

  SyncInfo sync_info_;

  du::Updater updater_;
  DiagnosedPublisher pd_imu_, pd_twist_, pd_mag_, pd_pres_, pd_temp_;
  ros::ServiceServer srv_tare_;

  ros::Publisher zOrientationPub_, xOrientationPub_, yOrientationPub_, zVelPub_;

  void FixImuRate();
  void LoadParameters();
  void CreateDiagnosedPublishers();
};

// Just don't like type that is ALL CAP
using VnErrorCode = VN_ERROR_CODE;
void VnEnsure(const VnErrorCode &error_code);

} // namespace imu_vn_100

#endif // IMU_VN_100_ROS_H_
