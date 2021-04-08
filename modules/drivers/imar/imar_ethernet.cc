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

#include "modules/drivers/imar/imar_ethernet.h"

#include <chrono>
#include <thread>

#include "modules/localization/msf/common/util/frame_transform.h"

/**
 * @namespace apollo::drivers::imar
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace imar {

bool ImarEthernet::Init() {
  // Create writers
  // links to topic "/apollo/localization/gps"
  gps_writer_ = node_->CreateWriter<apollo::localization::Gps>(
      imar_conf_.localization_topic());
  // links to topic "/apollo/sensor/gnss/corrected_imu"
  imu_writer_ = node_->CreateWriter<apollo::localization::CorrectedImu>(
      imar_conf_.imu_topic());
  // links to topic "/apollo/sensor/fortiss_imar_status"
  imar_status_writer_ = node_->CreateWriter<apollo::drivers::FortissImarStatus>(
      imar_conf_.imar_status_topic());
  // links to "/apollo/sensor/gnss/gnss_status"
  gnss_ins_status_writer_ = node_->CreateWriter<apollo::drivers::gnss::InsStat>(
      imar_conf_.ins_status_topic());

  // Publish status
  auto imar_status = std::make_shared<apollo::drivers::FortissImarStatus>();
  imar_status->set_type(apollo::drivers::FortissImarStatus_Type_CONNECTED);
  imar_status_writer_->Write(imar_status);

  return true;
}

//! start socket communication
bool ImarEthernet::ConfigureImar() {
  // initialize sdk
  p_ins_ = new insCom("iNatGPS");
  gps_data_ = new useUdpLogData();
  AINFO << "[ConfigureImar] iNat device: " << imar_conf_.ip_address()
            << " at TCP port " << imar_conf_.tcp_port() << " and UDP Port "
            << imar_conf_.udp_port() << ".";
  this->InitImar(this->p_ins_);
  return true;
}

bool ImarEthernet::Start() {
  if (!ConfigureImar()) {
    AERROR << "[Start] Could not start iNat communication.";
    return false;
  } else {
    AINFO << "[Start] iNat configuration done.";
  }

  //! start the spin thread
  imar_thread_ptr_.reset(new std::thread(&ImarEthernet::ImarSpin, this));
  AINFO << "[Start] Started iNat communication successfully.";
  return true;
}

void ImarEthernet::Stop() {
  p_ins_->closeSocket();
  p_ins_->closeUdpSocket();

  // Publish status
  auto imar_status = std::make_shared<apollo::drivers::FortissImarStatus>();
  imar_status->set_type(apollo::drivers::FortissImarStatus_Type_DISCONNECTED);
  imar_status_writer_->Write(imar_status);
}

std::tuple<double, double, double, double>
ImarEthernet::ConvertEulerAnglesToQuaternion(const double yaw,
                                             const double roll,
                                             const double pitch) {
  double cy = cos(-yaw * 0.5f);
  double sy = sin(-yaw * 0.5f);
  double cr = cos(roll * 0.5f);
  double sr = sin(roll * 0.5f);
  double cp = cos(pitch * 0.5f);
  double sp = sin(pitch * 0.5f);

  double qw = cy * cr * cp + sy * sr * sp;
  double qx = cy * sr * cp - sy * cr * sp;
  double qy = cy * cr * sp + sy * sr * cp;
  double qz = sy * cr * cp - cy * sr * sp;

  return std::make_tuple(qx, qy, qz, qw);
}

std::tuple<double, double, double> ImarEthernet::ConvertQuaternionToEulerAngles(
    const double qx, const double qy, const double qz, const double qw) {
  double yaw, roll, pitch;

  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (qw * qx + qy * qz);
  double cosr_cosp = +1.0 - 2.0 * (qx * qx + qy * qy);
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (qw * qy - qz * qx);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (qw * qz + qx * qy);
  double cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = atan2(siny_cosp, cosy_cosp);

  return std::make_tuple(yaw, roll, pitch);
}

void ImarEthernet::PublishSensorData() {
  //! GPS
  using apollo::localization::Gps;
  auto gps = std::make_shared<Gps>();
  double unix_sec_gps = cyber::Time::Now().ToSecond();
  gps->mutable_header()->set_timestamp_sec(unix_sec_gps);
  auto *pose = gps->mutable_localization();
  double qx, qy, qz, qw, roll, pitch, yaw;
  double posx, posy, posz;
  double vx, vy, vz;
  bool new_gnss_data = false;

  //  The INSRPY message contains the integration filter attitude solution in
  //  Euler representation (roll, pitch and yaw). The given Euler angles
  //  describe the orientation of the body frame with respect to the navigation
  //  frame (NED).
  if (imar_conf_.gps_available()) {  // Regular case: gps signal is available

    // check if the data is new: as there is no method to do this directly in
    // the sdk, we check long and lat. if the same -> skip
    if ((old_longitude_ != gps_data_->xcominsSol.dPos[0]) ||
        (old_lattitude_ != gps_data_->xcominsSol.dPos[1])) {
      old_longitude_ = gps_data_->xcominsSol.dPos[0];
      old_lattitude_ = gps_data_->xcominsSol.dPos[1];
      new_gnss_data = true;

      roll = gps_data_->xcominsSol.fRPY[0];
      pitch = gps_data_->xcominsSol.fRPY[1];
      yaw = gps_data_->xcominsSol.fRPY[2];

      vx = gps_data_->xcominsSol.fVel[0];  // x
      vy = gps_data_->xcominsSol.fVel[1];  // y
      vz = gps_data_->xcominsSol.fVel[2];  // z

      apollo::localization::msf::UTMCoor utm_xy;
      apollo::localization::msf::FrameTransform::LatlonToUtmXY(
          gps_data_->xcominsSol.dPos[0], gps_data_->xcominsSol.dPos[1],
          &utm_xy);

      posx = utm_xy.x;
      posy = utm_xy.y;
      posz = static_cast<double>(gps_data_->xcominsSol.fAlt);
    }

  } else { // irregular case, i.e. in garage
    new_gnss_data = true;

    if (previous_gps_message_ == nullptr) {
      previous_gps_message_ = std::make_shared<apollo::localization::Gps>();
    }

    if (previous_imu_message_ == nullptr) {
      previous_imu_message_ =
          std::make_shared<apollo::localization::CorrectedImu>();
    }

    double dt =
        previous_gps_message_->mutable_header()->timestamp_sec() - unix_sec_gps;

    auto *previous_pose = previous_gps_message_->mutable_localization();
    auto *previous_imu_pose =
        previous_imu_message_
            ->mutable_imu();  // required for linear acceleration

    // AERROR << "previous_pose->mutable_position()->x()" <<
    // previous_pose->mutable_position()->x(); AERROR <<
    // "previous_imu_pose->mutable_linear_acceleration()->x()" <<
    // previous_imu_pose->mutable_linear_acceleration()->x();

    posx =
        previous_pose->mutable_position()->x() +
        previous_pose->mutable_linear_velocity()->x() * dt +
        0.5 * previous_imu_pose->mutable_linear_acceleration()->x() * dt * dt;
    posy =
        previous_pose->mutable_position()->y() +
        previous_pose->mutable_linear_velocity()->y() * dt +
        0.5 * previous_imu_pose->mutable_linear_acceleration()->y() * dt * dt;
    posz =
        previous_pose->mutable_position()->z() +
        previous_pose->mutable_linear_velocity()->z() * dt +
        0.5 * previous_imu_pose->mutable_linear_acceleration()->z() * dt * dt;

    vx = previous_pose->mutable_linear_velocity()->x() +
         previous_imu_pose->mutable_linear_acceleration()->x() * dt;
    vy = previous_pose->mutable_linear_velocity()->y() +
         previous_imu_pose->mutable_linear_acceleration()->y() * dt;
    vz = previous_pose->mutable_linear_velocity()->z() +
         previous_imu_pose->mutable_linear_acceleration()->z() * dt;

    // get previous angles from quaternions
    auto *prev_quaternion = previous_pose->mutable_orientation();
    double prev_roll, prev_pitch, prev_yaw;

    tie(prev_roll, prev_pitch, prev_yaw) = ConvertQuaternionToEulerAngles(
        prev_quaternion->qx(), prev_quaternion->qy(), prev_quaternion->qz(),
        prev_quaternion->qw());

    roll =
        prev_roll +
        previous_imu_pose->mutable_angular_velocity()->x() *
            dt;  // +
                 // 0.5*previous_imu_pose->mutable_angular_acceleration()->x()*dt*dt;
    pitch =
        prev_pitch +
        previous_imu_pose->mutable_angular_velocity()->y() *
            dt;  // +
                 // 0.5*previous_imu_pose->mutable_angular_acceleration()->y()*dt*dt;
    yaw =
        prev_yaw +
        previous_imu_pose->mutable_angular_velocity()->z() *
            dt;  // +
                 // 0.5*previous_imu_pose->mutable_angular_acceleration()->z()*dt*dt;
  }

  if (new_gnss_data) {

//So setzt apollo die quaternion:
// add "@eigen", in BUILD
  // // 2. orientation
  // Eigen::Quaterniond q =
  //     Eigen::AngleAxisd(ins->euler_angles().z() - 90 * M_PI / 180.0,
  //                       Eigen::Vector3d::UnitZ()) *
  //     Eigen::AngleAxisd(-ins->euler_angles().y(), Eigen::Vector3d::UnitX()) *
  //     Eigen::AngleAxisd(ins->euler_angles().x(), Eigen::Vector3d::UnitY());
  // qx = q.x;
  // qy = q.y;
  // qz = q.z;
  // qw = q.w;



    tie(qx, qy, qz, qw) = ConvertEulerAnglesToQuaternion(yaw, roll, pitch);

    pose->mutable_orientation()->set_qx(qx);
    pose->mutable_orientation()->set_qy(qy);
    pose->mutable_orientation()->set_qz(qz);
    pose->mutable_orientation()->set_qw(qw);

    pose->mutable_position()->set_x(posx);  // x
    pose->mutable_position()->set_y(posy);  // y
    pose->mutable_position()->set_z(posz);  // z

    pose->mutable_linear_velocity()->set_x(vx);  // x
    pose->mutable_linear_velocity()->set_y(vy);  // y
    pose->mutable_linear_velocity()->set_z(vz);  // z

    if (!gps_writer_->Write(gps)) {
      AERROR << "failed to send gps message!";
    }
  }

  //! IMU
  using apollo::localization::CorrectedImu;
  auto imu = std::make_shared<CorrectedImu>();
  double unix_sec_imu = cyber::Time::Now().ToSecond();
  imu->mutable_header()->set_timestamp_sec(unix_sec_imu);
  auto *imu_pose = imu->mutable_imu();
  bool new_imu_data = false;

  if (imar_conf_.gps_available()) {  // Regular case: gps signal is available
    if ((old_acceleration_[0] != gps_data_->xcominsSol.fAcc[0]) ||
        (old_acceleration_[1] != gps_data_->xcominsSol.fAcc[1]) ||
        (old_acceleration_[2] != gps_data_->xcominsSol.fAcc[2])) {

      new_imu_data = true;
      old_acceleration_[0] = gps_data_->xcominsSol.fAcc[0];
      old_acceleration_[1] = gps_data_->xcominsSol.fAcc[1];
      old_acceleration_[2] = gps_data_->xcominsSol.fAcc[2];

      // Rotation matrix to rotate acc vector and angular velocity vector from 
      // body frame to ENU frame

      // Rotation matrix
      roll = gps_data_->xcominsSol.fRPY[0];
      pitch = gps_data_->xcominsSol.fRPY[1];
      yaw = gps_data_->xcominsSol.fRPY[2];
      //TODO in the matlab proof of concept I set pitch and roll to zero!
      const double st[3] = {sin(yaw), sin(pitch), sin(roll)};
      const double ct[3] = {cos(yaw), cos(pitch), cos(roll)};
      double R11, R12, R13, R21, R22, R23, R31, R32, R33;
      R11 = ct[1]*ct[0];
      R12 = st[2]*st[1]*ct[0] - ct[2]*st[0];
      R13 = ct[2]*st[1]*ct[0] + st[2]*st[0];
      R21 = ct[1]*st[0];
      R22 = st[2]*st[1]*st[0] + ct[2]*ct[0];
      R23 = ct[2]*st[1]*st[0] - st[2]*ct[0];
      R31 = -st[1];
      R32 = st[2]*ct[1];
      R33 = ct[2]*ct[1];

      // NOTE the -y here!
      const double a[] = {gps_data_->xcominsSol.fAcc[0], 
                          -gps_data_->xcominsSol.fAcc[1], 
                          gps_data_->xcominsSol.fAcc[2]};

      // Rotated Acceleration
      const double a_trans_x = R11*a[0] + R12*a[1] + R13*a[2];
      const double a_trans_y = R21*a[0] + R22*a[1] + R31*a[2];
      const double a_trans_z = R31*a[0] + R32*a[1] + R33*a[2];

      // TODO I am not sure about this transformation, I here atm only
      // flip z
      const double ang_vel_trans_x = gps_data_->xcominsSol.fOmg[0];
      const double ang_vel_trans_y = gps_data_->xcominsSol.fOmg[1];
      const double ang_vel_trans_z = -gps_data_->xcominsSol.fOmg[2];
 
      // Fill msg   
      imu_pose->mutable_angular_velocity()->set_x(ang_vel_trans_x);
      imu_pose->mutable_angular_velocity()->set_y(ang_vel_trans_y);
      imu_pose->mutable_angular_velocity()->set_z(ang_vel_trans_z);

      imu_pose->mutable_linear_acceleration()->set_x(a_trans_x);
      imu_pose->mutable_linear_acceleration()->set_y(a_trans_y);
      imu_pose->mutable_linear_acceleration()->set_z(a_trans_z);

      imu_pose->mutable_euler_angles()->set_x(roll);
      imu_pose->mutable_euler_angles()->set_y(pitch);
      imu_pose->mutable_euler_angles()->set_z(yaw);
    }
  } else { // irregular case, i.e. in garage
    new_imu_data = true;

    imu_pose->mutable_angular_velocity()->set_x(
        gps_data_->xcomimuComp.fOmg[0]);  // x
    imu_pose->mutable_angular_velocity()->set_y(
        gps_data_->xcomimuComp.fOmg[1]);  // y
    imu_pose->mutable_angular_velocity()->set_z(
        gps_data_->xcomimuComp.fOmg[2]);  // z

    imu_pose->mutable_linear_acceleration()->set_x(
        gps_data_->xcomimuComp.fAcc[0]);  // x
    imu_pose->mutable_linear_acceleration()->set_y(
        gps_data_->xcomimuComp.fAcc[1]);  // y
    imu_pose->mutable_linear_acceleration()->set_z(
        gps_data_->xcomimuComp.fAcc[2]);  // z
  }

  if (new_imu_data) {
    if (!imu_writer_->Write(imu)) {
      AERROR << "failed to send imu message!";
    }
  }

  if (new_gnss_data || new_imu_data) {
    auto ins_status = std::make_shared<apollo::drivers::gnss::InsStat>();
    ins_status->mutable_header()->set_timestamp_sec(unix_sec_imu);
    // TODO: also set status fields?
    gnss_ins_status_writer_->Write(ins_status);
  }

  // save messages
  previous_gps_message_ = gps;
  previous_imu_message_ = imu;
}

void ImarEthernet::ImarSpin() {
  while (cyber::OK()) {
    //! read data from UDP socket
    if (-1 != p_ins_->getUdpUseData(gps_data_)) {
      if (true) {
        if (old_frame_count_ != gps_data_->xcominsSol.tHeader.ucFrameCnt) {
          old_frame_count_ = gps_data_->xcominsSol.tHeader.ucFrameCnt;
          // AERROR << "old frame cout is: " << old_frame_count_ <<"\n"; //TODO
          // AINFO << "Publishing NEW sensor data \n";
          PublishSensorData();
        }
      }
    }
  }
}

bool ImarEthernet::OnError(const std::string &error_msg) {
  AERROR << error_msg;
  return false;
}

bool ImarEthernet::InitImar(insCom *p_ins) {
  //! Open TCP Connection
  if (-1 == p_ins->openNConnectSocket(imar_conf_.ip_address().c_str(),
                                      imar_conf_.tcp_port())) {
    AERROR << "[InitImar] Could not connect via TCP";
    return false;
  }

  //! Create TCP Receiver Thread
  thread insComThread(&insCom::parseRxRunData, p_ins);
  insComThread.detach();

  //! Open UDP Connection (PC-IP is not needed within the function; thus
  //! nullptr)
  if (-1 == p_ins->ConnectUDPSocket(nullptr, imar_conf_.udp_port())) {
    AERROR << "[InitImar] Could not connect via UDP";
    return false;
  }

  //! Create UDP Receiver Thread
  thread udpComThread(&insCom::parseRxRunUdpData, p_ins);
  udpComThread.detach();

  return true;
}

}  // namespace imar
}  // namespace drivers
}  // namespace apollo
