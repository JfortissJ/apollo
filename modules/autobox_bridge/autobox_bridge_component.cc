/******************************************************************************
 * Copyright 2020 fortiss GmbH
 * Authors: Tobias Kessler, Jianjie Lin
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
#include "modules/autobox_bridge/autobox_bridge_component.h"

#include <time.h>
namespace apollo {
namespace autobox_bridge {

using apollo::common::time::Clock;
using apollo::control::ControlCommand;
using apollo::cyber::proto::RoleAttributes;
using apollo::drivers::AutoboxBridgeConf;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

AutoBoxBridgeComponent::AutoBoxBridgeComponent()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::CANBUS),
      new_trajectory_(false),
      new_localization_(false),
      new_control_cmd_(false),
      check_timeouts_(false) {}

bool AutoBoxBridgeComponent::Init() {
  // Get Parameters
  AutoboxBridgeConf abx_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &abx_config)) {
    monitor_logger_buffer_.ERROR("Unable to load autobox gateway conf file: " +
                                 config_file_path_);
    return false;
  }
  check_timeouts_ = abx_config.check_timeouts();
  cyber::ReaderConfig reader_config_traj;
  reader_config_traj.channel_name = abx_config.trajectory_channel_name();
  reader_config_traj.pending_queue_size = 100;
  trajectory_reader_ = node_->CreateReader<ADCTrajectory>(
      reader_config_traj,
      [&](const std::shared_ptr<ADCTrajectory>& trajectory_msg) {
        trajectoryMsgCallback(trajectory_msg);
      });

  // Establish cyber connection localization
  cyber::ReaderConfig reader_config_loca;
  reader_config_loca.channel_name = abx_config.localization_channel_name();
  reader_config_loca.pending_queue_size = 100;
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      reader_config_loca,
      [&](const std::shared_ptr<LocalizationEstimate>& localization_msg) {
        localizationMsgCallback(localization_msg);
      });

  // Establish cyber connection control command
  cyber::ReaderConfig reader_config_control_cmd;
  reader_config_control_cmd.channel_name = abx_config.controlcmd_channel_name();
  reader_config_control_cmd.pending_queue_size = 100;
  control_cmd_reader_ = node_->CreateReader<apollo::control::ControlCommand>(
      reader_config_control_cmd,
      [&](const std::shared_ptr<apollo::control::ControlCommand>&control_cmd_msg) 
      { 
	    
      controlCmdMsgCallback(control_cmd_msg); 
      }
      );

  // Estabilish cyber connection chassis command
  cyber::ReaderConfig reader_config_chassis_cmd;
  reader_config_chassis_cmd.channel_name = abx_config.chassis_channel_name();
  reader_config_chassis_cmd.pending_queue_size = 100;
  chassis_cmd_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
      reader_config_chassis_cmd,
      [this](const std::shared_ptr<apollo::canbus::Chassis>& chassis_) {
        std::lock_guard<std::mutex> lock(chassis_cmd_mutex_);
        chassis_cmd_->CopyFrom(*chassis_);
      });

  monitor_logger_buffer_.INFO("Started AutoBoxUDP connection.");

  // Initialized stored trajectory and localization and control command
  trajectory_ = std::make_shared<ADCTrajectory>();
  trajectory_->mutable_header()->set_timestamp_sec(0.0);
  localization_ =
      std::make_shared<apollo::localization::LocalizationEstimate>();
  localization_->mutable_header()->set_timestamp_sec(0.0);
  control_cmd_ = std::make_shared<apollo::control::ControlCommand>();
  control_cmd_->mutable_header()->set_timestamp_sec(0.0);
  chassis_cmd_ = std::make_shared<apollo::canbus::Chassis>();
  chassis_cmd_->mutable_header()->set_timestamp_sec(0.0);

  chassis_writer_ = node_->CreateWriter<apollo::canbus::ChassisToAutoboxBridge>(
      "/apollo/autobox_bridge/chassis");
  control_cmd_writer_ =
      node_->CreateWriter<apollo::control::ControlCommandToAutoboxBridge>(
          "/apollo/autobox_bridge/control");
  localization_writer_ =
      node_->CreateWriter<apollo::localization::LocalizationToAutoboxBridge>(
          "/apollo/autobox_bridge/localization");
  trajectory_writer_ =
      node_->CreateWriter<apollo::planning::ADCTrajectoryToAutoboxBridge>(
          "/apollo/autobox_bridge/adctrajectory");
  monitor_logger_buffer_.INFO("AutoBridgeComponent successfully initialized");

  return true;
}
bool AutoBoxBridgeComponent::Proc() {
  publishTrajectory();
  publishControl();
  publishLocalization();
  publishChassis();
  return true;
}
void AutoBoxBridgeComponent::trajectoryMsgCallback(
    const std::shared_ptr<apollo::planning::ADCTrajectory>& trajectory_msg) {
  AINFO << "Got a trajectory at t= " << apollo::cyber::Time::Now().ToSecond();
  if (checkTrajectoryMsg(trajectory_msg)) {
    //trajectory_mutex_.lock();
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory_->CopyFrom(*trajectory_msg);
    new_trajectory_ = true;
    //trajectory_mutex_.unlock();
  }
}

void AutoBoxBridgeComponent::localizationMsgCallback(
    const std::shared_ptr<apollo::localization::LocalizationEstimate>&
        localization_msg) {
  AINFO << "Got a localization at t= " << apollo::cyber::Time::Now().ToSecond();
  if (checkLocalizationMsg(localization_msg)) {
    //localization_mutex_.lock();
    std::lock_guard<std::mutex> lock(localization_mutex_);
    localization_->CopyFrom(*localization_msg);
    new_localization_ = true;
    //localization_mutex_.unlock();
  }
}
void AutoBoxBridgeComponent::controlCmdMsgCallback(
    const std::shared_ptr<apollo::control::ControlCommand>& control_cmd_msg) {
  AINFO << "Got a control command at t= "
        << apollo::cyber::Time::Now().ToSecond();
  if (checkControlCmdMsg(control_cmd_msg)) {
    //control_cmd_mutex_.lock();
    //control_cmd_ = control_cmd_msg;
    std::lock_guard<std::mutex> lock(control_cmd_mutex_);
    control_cmd_->CopyFrom(*control_cmd_msg);
    new_control_cmd_ = true;
    //control_cmd_mutex_.unlock();
  }
}
bool AutoBoxBridgeComponent::checkTrajectoryMsg(
    const std::shared_ptr<apollo::planning::ADCTrajectory>& trajectory_msg) {
  // msg is available
  if (trajectory_msg == NULL) {
    AERROR << "Received empty trajectory msg!";
    return false;
  }

  // msg is new (compared to the old, stored one)
  if (trajectory_->mutable_header()->timestamp_sec() >
      trajectory_msg->mutable_header()->timestamp_sec()) {
    AERROR << "Trajectory msg is older than previously received trajectory!";
    AERROR << "old traj " << trajectory_->mutable_header()->timestamp_sec();
    AERROR << "new traj " << trajectory_msg->mutable_header()->timestamp_sec();
    return false;
  }

  // msg is valid
  if (!trajectory_msg->estop().is_estop() &&
      trajectory_msg->trajectory_point_size() == 0) {
    AERROR_EVERY(100) << "planning has no trajectory point.";
    return false;
  }

  // msg timed out
  //const double timedelta = 100.0;  // todo define a reasonable value here!
  //if (check_timeouts_ &&
  //    cyber::Time::Now().ToSecond() -
  //            trajectory_msg->mutable_header()->timestamp_sec() >
  //        timedelta) {  // check if timestamp is recent
  //  AERROR_EVERY(100) << "trajectory message timed out.";
  //  return false;
 // }

  // no error
  return true;
}

bool AutoBoxBridgeComponent::checkLocalizationMsg(
    const std::shared_ptr<apollo::localization::LocalizationEstimate>&
        localization_msg) {
  // msg is available
  if (localization_msg == NULL) {
    AERROR << "Received empty localization msg!";
    return false;
  }

  // msg is valid
  if (!(localization_msg->has_header() && localization_msg->has_pose() &&
        localization_msg->mutable_pose()->has_position() &&
        localization_msg->mutable_pose()->has_orientation() &&
        localization_msg->mutable_pose()->has_linear_velocity() &&
        localization_msg->mutable_pose()->has_linear_acceleration())) {
    AERROR << "Localization message invalid! at least one field is missing!";
    return false;
  }
  if (std::isnan(localization_msg->mutable_pose()->mutable_position()->x()) ||
      std::isnan(localization_msg->mutable_pose()->mutable_position()->y()) ||
      std::isnan(localization_msg->mutable_pose()->mutable_position()->z()) ||
      std::isnan(
          localization_msg->mutable_pose()->mutable_orientation()->qw()) ||
      std::isnan(
          localization_msg->mutable_pose()->mutable_orientation()->qx()) ||
      std::isnan(
          localization_msg->mutable_pose()->mutable_orientation()->qy()) ||
      std::isnan(
          localization_msg->mutable_pose()->mutable_orientation()->qz())) {
    AERROR << "Localization message invalid! at least one field is NaN!";
    return false;
  }

  // msg is new (compared to the old, stored one)
  if (localization_->mutable_header()->timestamp_sec() >
      localization_msg->mutable_header()->timestamp_sec()) {
    AERROR
        << "Localization msg is older than previously received localization!";
    AERROR << "Old Loca: " << localization_->mutable_header()->timestamp_sec();
    AERROR << "New Loca: "
           << localization_msg->mutable_header()->timestamp_sec();
    return false;
  }

  // msg timed out
  //const double timedelta = 100.0;  // todo define a reasonable value here!
  //if (check_timeouts_ &&
  //    cyber::Time::Now().ToSecond() -
  //            localization_msg->mutable_header()->timestamp_sec() >
  //        timedelta) {  // check if timestamp is recent
  //  AERROR_EVERY(100) << "Localization message timed out.";
  //  return false;
 // }

  // no error
  return true;
}

bool AutoBoxBridgeComponent::checkControlCmdMsg(
    const std::shared_ptr<apollo::control::ControlCommand>& control_cmd_msg) {
  // msg is available
  if (control_cmd_msg == NULL) {
    AERROR << "Received empty control command msg!";
    return false;
  }
  // msg is valid
  // if (!(control_cmd_msg->has_header() && control_cmd_msg->has_steering_rate()
  // &&
  //       control_cmd_msg->has_steering_target() &&
  //       control_cmd_msg->has_acceleration() && control_cmd_msg->has_brake()
  //       && control_cmd_msg->has_throttle())) {
  //   AERROR << "control command message invalid! at least one field is
  //   missing!"; return false;
  // }
  if (!control_cmd_msg->has_header()) {
    AERROR << "control command message invalid! header is missing!";
    return false;
  }
//  if (!control_cmd_msg->has_steering_rate()) {
//    AERROR << "control command messgae invalid. steearing rate is missing";
//    return false;
//  }
  if (!control_cmd_msg->has_throttle()) {
    AERROR << "control command messgae invalid. throttle  is missing";
    return false;
  }
  if (!control_cmd_msg->has_brake()) {
    AERROR << "control command messgae invalid. brake  is missing";
    return false;
  }
  //if (!control_cmd_msg->has_steering_target()) {
  //  AERROR << "control command messgae invalid. steering_target  is missing";
  //  return false;
  //}
  // if (!control_cmd_msg->has_acceleration()) {
  //   AERROR << "control command messgae invalid. acceleration  is missing";
  //   return false;
  // }
  if (std::isnan(control_cmd_msg->acceleration()) ||
      std::isnan(control_cmd_msg->brake()) ||
      std::isnan(control_cmd_msg->throttle()) ||
      std::isnan(control_cmd_msg->steering_rate()) ||
      std::isnan(control_cmd_msg->steering_target())) {
    AERROR << "Control Command message invalid! at least one field is NaN!";
    return false;
  }
  // msg is new (compared to the old, stored one)
  if (control_cmd_->mutable_header()->timestamp_sec() >
      control_cmd_msg->mutable_header()->timestamp_sec()) {
    AERROR << "control command msg is older than previously received control "
              "command!";
    AERROR << "Old control command: "
           << control_cmd_->mutable_header()->timestamp_sec();
    AERROR << "New control command: "
           << control_cmd_msg->mutable_header()->timestamp_sec();
    return false;
  }
  // msg timed out
  //const double timedelta = 100.0;  // todo define a reasonable value here!
  //if (check_timeouts_ &&
  //    cyber::Time::Now().ToSecond() -
  //            control_cmd_msg->mutable_header()->timestamp_sec() >
  //        timedelta) {
    // check if timestamp is recent<
  //  AERROR_EVERY(100) << "Localization message timed out.";
  //  return false;
  //}
  return true;
}
void AutoBoxBridgeComponent::publishTrajectory() {
  apollo::planning::ADCTrajectoryToAutoboxBridge adctrajectorytoAutoboxBridge;
  adctrajectorytoAutoboxBridge.mutable_header()->CopyFrom(
      trajectory_->header());
  std::vector<common::TrajectoryPoint> pb_trajectory_;
  pb_trajectory_.assign(trajectory_->trajectory_point().begin(),
                        trajectory_->trajectory_point().end());
  for (int i = 0; i < trajectory_->trajectory_point().size(); i++) {
    apollo::common::TrajectoryPoint* pt_traj_points =
        adctrajectorytoAutoboxBridge.add_trajectory_point();
    pt_traj_points->CopyFrom(pb_trajectory_[i]);
    /* // if (pb_trajectory_[i].path_point().has_x()) {
    //   pt_traj_points->path_point().set_x(pb_trajectory_[i].path_point().x());
    // }

    // if (pb_trajectory_[i].path_point().has_y()) {
    // pt_traj_points->path_point().set_y(pb_trajectory_[i].path_point().y());
    // }

    // if (pb_trajectory_[i].path_point().has_z()) {
    // pt_traj_points->path_point().set_z(pb_trajectory_[i].path_point().z());
    // }

    // if (pb_trajectory_[i].path_point().has_theta()) {
    //
    pt_traj_points->path_point().set_theta(pb_trajectory_[i].path_point().theta());
    // }

    // if (pb_trajectory_[i].path_point().has_kappa()) {
    //
    pt_traj_points->path_point().set_kappa(pb_trajectory_[i].path_point().kappa());
    // }

    // if (pb_trajectory_[i].path_point().has_s()) {
    // pt_traj_points->path_point().set_s(pb_trajectory_[i].path_point().s());
    // }

    // if (pb_trajectory_[i].path_point().has_dkappa()) {
    //
    pt_traj_points->path_point().set_dkappa(pb_trajectory_[i].path_point().dkappa());
    // }

    // if (pb_trajectory_[i].path_point().has_ddkappa()) {
    //
    pt_traj_points->path_point().set_ddkappa(pb_trajectory_[i].path_point().ddkappa());
    // }

    // Attention: although lane_id is optional in proto-definition, proto-c does
    // not protive a has_lane_id member --> currently not supported in this
    // converter
    // pt_traj_points->path_point().has_lane_id =
    // pb_trajectory_[i].path_point().has_lane_id()); if
    // (pb_trajectory_[i].path_point().has_lane_id()) {
    //    pt_traj_points->path_point().lane_id =
    //    pb_trajectory_[i].path_point().lane_id());
    //}

    // if (pb_trajectory_[i].has_v()) {
    //   pt_traj_points->set_v(pb_trajectory_[i].v());
    //     }

    //     if (pb_trajectory_[i].has_a()) {
    //         pt_traj_points->set_a(pb_trajectory_[i].a());
    //     }

    //     if (pb_trajectory_[i].has_relative_time()) {
    // pt_traj_points->set_relative_time(pb_trajectory_[i].relative_time());
    //     }
        */
  }
  trajectory_writer_->Write(adctrajectorytoAutoboxBridge);
}
void AutoBoxBridgeComponent::publishLocalization() {
  apollo::localization::LocalizationToAutoboxBridge localizationToAutoboxBridge;
  localizationToAutoboxBridge.mutable_header()->CopyFrom(
      localization_->header());
  localizationToAutoboxBridge.set_measurement_time(
      localization_->measurement_time());
  localizationToAutoboxBridge.mutable_pose()->CopyFrom(localization_->pose());
  localizationToAutoboxBridge.mutable_uncertainty()->CopyFrom(
      localization_->uncertainty());
  /*
  apollo::localization::Pose* pb_pose_=new apollo::localization::Pose();
  apollo::common::PointENU* point_enu_ = new apollo::common::PointENU();
  point_enu_->set_x(localization_->pose().position().x());
  point_enu_->set_y(localization_->pose().position().y());
  point_enu_->set_z(localization_->pose().position().z());
  pb_pose_->set_allocated_position(point_enu_);

  apollo::common::Quaternion* quanternion_ = new apollo::common::Quaternion();
  quanternion_->set_qw(localization_->pose().orientation().qw());
  quanternion_->set_qx(localization_->pose().orientation().qx());
  quanternion_->set_qy(localization_->pose().orientation().qy());
  quanternion_->set_qz(localization_->pose().orientation().qz());
  pb_pose_->set_allocated_orientation(quanternion_);

  apollo::common::Point3D* linear_velocity_ = new apollo::common::Point3D();
  linear_velocity_->set_x(localization_->pose().linear_velocity().x());
  linear_velocity_->set_y(localization_->pose().linear_velocity().y());
  linear_velocity_->set_z(localization_->pose().linear_velocity().z());
  pb_pose_->set_allocated_linear_velocity(linear_velocity_);

  apollo::common::Point3D* linear_acceleration_ = new apollo::common::Point3D();
  linear_acceleration_->set_x(localization_->pose().linear_acceleration().x());
  linear_acceleration_->set_y(localization_->pose().linear_acceleration().y());
  linear_acceleration_->set_z(localization_->pose().linear_acceleration().z());
  pb_pose_->set_allocated_linear_acceleration(linear_acceleration_);

  apollo::common::Point3D* angular_velocity_ = new apollo::common::Point3D();
  angular_velocity_->set_x(localization_->pose().angular_velocity().x());
  angular_velocity_->set_y(localization_->pose().angular_velocity().y());
  angular_velocity_->set_z(localization_->pose().angular_velocity().z());
  pb_pose_->set_allocated_angular_velocity(angular_velocity_);

  apollo::common::Point3D* linear_acceleration_vrf_ = new
  apollo::common::Point3D();
  linear_acceleration_vrf_->set_x(localization_->pose().linear_acceleration_vrf().x());
  linear_acceleration_vrf_->set_y(localization_->pose().linear_acceleration_vrf().y());
  linear_acceleration_vrf_->set_z(localization_->pose().linear_acceleration_vrf().z());
  pb_pose_->set_allocated_linear_acceleration_vrf(linear_acceleration_vrf_);


  apollo::common::Point3D* angular_velocity_vrf = new apollo::common::Point3D();
  angular_velocity_vrf->set_x(localization_->pose().angular_velocity_vrf().x());
  angular_velocity_vrf->set_y(localization_->pose().angular_velocity_vrf().y());
  angular_velocity_vrf->set_z(localization_->pose().angular_velocity_vrf().z());
  pb_pose_->set_allocated_angular_velocity_vrf(angular_velocity_vrf);

  apollo::common::Point3D* euler_angles = new apollo::common::Point3D();
  euler_angles->set_x(localization_->pose().euler_angles().x());
  euler_angles->set_y(localization_->pose().euler_angles().y());
  euler_angles->set_z(localization_->pose().euler_angles().z());
  pb_pose_->set_allocated_euler_angles(euler_angles);

  pb_pose_->set_heading(localization_->pose().heading());

  apollo::localization::Uncertainty* pb_uncertainty_=new
  apollo::localization::Uncertainty();

  apollo::common::Point3D* position_std_dev = new apollo::common::Point3D();
  position_std_dev->set_x(localization_->uncertainty().position_std_dev().x());
  position_std_dev->set_y(localization_->uncertainty().position_std_dev().y());
  position_std_dev->set_z(localization_->uncertainty().position_std_dev().z());
  pb_uncertainty_->set_allocated_position_std_dev(position_std_dev);

  apollo::common::Point3D* orientation_std_dev = new apollo::common::Point3D();
  orientation_std_dev->set_x(localization_->uncertainty().orientation_std_dev().x());
  orientation_std_dev->set_y(localization_->uncertainty().orientation_std_dev().y());
  orientation_std_dev->set_z(localization_->uncertainty().orientation_std_dev().z());
  pb_uncertainty_->set_allocated_orientation_std_dev(orientation_std_dev);

  apollo::common::Point3D* linear_velocity_std_dev = new
  apollo::common::Point3D();
  linear_velocity_std_dev->set_x(localization_->uncertainty().linear_velocity_std_dev().x());
  linear_velocity_std_dev->set_y(localization_->uncertainty().linear_velocity_std_dev().y());
  linear_velocity_std_dev->set_z(localization_->uncertainty().linear_velocity_std_dev().z());
  pb_uncertainty_->set_allocated_linear_velocity_std_dev(linear_velocity_std_dev);

  apollo::common::Point3D* linear_acceleration_std_dev = new
  apollo::common::Point3D();
  linear_acceleration_std_dev->set_x(localization_->uncertainty().linear_acceleration_std_dev().x());
  linear_acceleration_std_dev->set_y(localization_->uncertainty().linear_acceleration_std_dev().y());
  linear_acceleration_std_dev->set_z(localization_->uncertainty().linear_acceleration_std_dev().z());
  pb_uncertainty_->set_allocated_linear_acceleration_std_dev(linear_acceleration_std_dev);

  apollo::common::Point3D* angular_velocity_std_dev = new
  apollo::common::Point3D();
  angular_velocity_std_dev->set_x(localization_->uncertainty().angular_velocity_std_dev().x());
  angular_velocity_std_dev->set_y(localization_->uncertainty().angular_velocity_std_dev().y());
  angular_velocity_std_dev->set_z(localization_->uncertainty().angular_velocity_std_dev().z());
  pb_uncertainty_->set_allocated_angular_velocity_std_dev(angular_velocity_std_dev);

  localizationToAutoboxBridge.set_allocated_uncertainty(pb_uncertainty_);
  //localizationToAutoboxBridge.set_allocated_pose(pb_pose_);*/
  localization_writer_->Write(localizationToAutoboxBridge);
}
void AutoBoxBridgeComponent::publishControl() {
  apollo::control::ControlCommandToAutoboxBridge controlcommandtoAutoBoxBridge;
  controlcommandtoAutoBoxBridge.mutable_header()->CopyFrom(control_cmd_->header());
  controlcommandtoAutoBoxBridge.set_brake(control_cmd_->brake());
  controlcommandtoAutoBoxBridge.set_throttle(control_cmd_->throttle());
  //controlcommandtoAutoBoxBridge.set_steering_rate(control_cmd_->steering_rate());
  controlcommandtoAutoBoxBridge.set_steering_rate(0.5);
  controlcommandtoAutoBoxBridge.set_steering_target(5);
  //controlcommandtoAutoBoxBridge.set_steering_target(control_cmd_->steering_target());
  //controlcommandtoAutoBoxBridge.set_acceleration(control_cmd_->acceleration());
  controlcommandtoAutoBoxBridge.set_acceleration(10);
  control_cmd_writer_->Write(controlcommandtoAutoBoxBridge);
}
void AutoBoxBridgeComponent::publishChassis() {
  apollo::canbus::ChassisToAutoboxBridge chassistoAutoboxBridge;
  srand(time(NULL));
  int count = 10000;
  int i = rand() % count;
  float total = static_cast<float>(count);
  float hundred = 100.00;
  auto pb_msg = std::make_shared<apollo::canbus::Chassis>();
  double timestamp_ = Clock::NowInSeconds();
  float coefficient = static_cast<float>(i);
  std::cout<<"the sequence_num: "<<i<<std::endl;
  std::cout<<"the timestamp_: "<<timestamp_<<std::endl;
  std::cout<<"the engine_rpm: "<<static_cast<float>(coefficient * 2.0)<<std::endl;
  std::cout<<"the odometer_m: "<<static_cast<float>(coefficient * 1.0)<<std::endl;
  chassistoAutoboxBridge.mutable_header()->set_sequence_num(i);
  chassistoAutoboxBridge.mutable_header()->set_timestamp_sec(timestamp_);
  chassistoAutoboxBridge.set_engine_started(true);
  chassistoAutoboxBridge.set_engine_rpm(static_cast<float>(coefficient * 2.0));
  chassistoAutoboxBridge.set_odometer_m(coefficient);
  chassistoAutoboxBridge.set_fuel_range_m(100);
  chassistoAutoboxBridge.set_throttle_percentage(coefficient * hundred / total);
  chassistoAutoboxBridge.set_brake_percentage(coefficient * hundred / total);
  chassistoAutoboxBridge.set_steering_percentage(static_cast<double>(coefficient * hundred / total));
  chassistoAutoboxBridge.set_steering_torque_nm(coefficient);
  chassistoAutoboxBridge.set_parking_brake(i % 2);
  chassis_writer_->Write(chassistoAutoboxBridge);
}
}  // namespace autobox_bridge
}  // namespace apollo
