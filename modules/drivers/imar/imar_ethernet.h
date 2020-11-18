/******************************************************************************
 * Copyright 2020 fortiss GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_DRIVERS_IMAR_IMAR_ETHERNET_H_
#define MODULES_DRIVERS_IMAR_IMAR_ETHERNET_H_

#include <float.h>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/imar/imar_sdk/insClass.h"
#include "modules/drivers/imar/proto/imar_conf.pb.h"
#include "modules/drivers/proto/fortiss_hardware_status.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/pose.pb.h"

/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace imar {

// Todo maybe we need a logger also here and not only in the imar component?
class ImarEthernet {
 public:
  ImarEthernet(const std::shared_ptr<apollo::cyber::Node>& node,
               const imarConf& imar_conf)
      : imar_conf_(imar_conf),
        node_(node),
        old_frame_count_(UINT8_MAX),
        old_longitude_(DBL_MAX),
        old_lattitude_(DBL_MAX) {
    old_acceleration_[0] = DBL_MAX;
    old_acceleration_[1] = DBL_MAX;
    old_acceleration_[2] = DBL_MAX;
  }

  ~ImarEthernet() {}

  bool Init();

  bool Start();

  void Stop();

  void PublishSensorData();

  void set_gps_data(useUdpLogData* gps_data) { gps_data_ = gps_data; }

 private:
  bool OnError(const std::string& error_msg);

  bool InitImar(insCom* p_ins);  //! iNat start

  bool ConfigureImar();

  void ImarSpin();

  std::tuple<double, double, double, double> ConvertEulerAnglesToQuaternion(
      const double yaw, const double roll, const double pitch);
  std::tuple<double, double, double> ConvertQuaternionToEulerAngles(
      const double qx, const double qy, const double qz, const double qw);

  imarConf imar_conf_;
  insCom* p_ins_;
  useUdpLogData* gps_data_;
  std::unique_ptr<std::thread> imar_thread_ptr_;

  std::shared_ptr<apollo::localization::Gps> previous_gps_message_ = nullptr;
  ;
  std::shared_ptr<apollo::localization::CorrectedImu> previous_imu_message_ =
      nullptr;

  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::localization::Gps>>
      gps_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::localization::CorrectedImu>>
      imu_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::FortissImarStatus>>
      imar_status_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::gnss::InsStat>>
      gnss_ins_status_writer_ = nullptr;

  uint8_t old_frame_count_;  //! for gps_data_->xcominsSol.Header.ucFrameCnt.
                             //! check if a message is really new
  double old_longitude_;  //! for gps_data_->xcominssol.fPos[0]. check if long
                          //! or lat has changed and do not publish
                          //! localization::gps data in case not
  double old_lattitude_;  //! for gps_data_->xcominssol.fPos[1]. check if long
                          //! or lat has changed and do not publish
                          //! localization::gps data in case not
  double old_acceleration_[3];  //! for gps_data_->xcominsSol.fAcc. check if the
                                //! acceleration values have changed and do not
                                //! publish localization::correctedImu data in
                                //! case not
};

}  // namespace imar
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_IMAR_IMAR_ETHERNET_H_
