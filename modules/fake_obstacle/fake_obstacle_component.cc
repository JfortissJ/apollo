/******************************************************************************
 * Copyright 2021 fortiss GmbH
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

#include "modules/fake_obstacle/fake_obstacle_component.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace fake_obstacle {

// using apollo::hdmap::HDMapUtil;
using apollo::canbus::Chassis;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ReferenceLineProvider;
using apollo::routing::RoutingResponse;

FakeObstacleComponent::FakeObstacleComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING),
      s_init_obstacle_(0.0),
      t_init_obstacle_(0.0),
      obstacle_has_started_(false) {
  AERROR << "Started FakeObstacleComponent !";
}

FakeObstacleComponent::~FakeObstacleComponent() {
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
}

bool FakeObstacleComponent::Init() {
  // load proto file
  if (!cyber::common::GetProtoFromFile(config_file_path_, &fake_obst_config_)) {
    monitor_logger_buffer_.ERROR("Unable to load fake obstacle conf file: " +
                                 config_file_path_);
  }

  // initialize all readers and writers
  if (!InitReaders()) {
    AERROR << "Could not initialize readers";
    return false;
  }

  obstacle_writer_ =
      node_->CreateWriter<PerceptionObstacles>(FLAGS_perception_obstacle_topic);

  // load map
  hdmap_ = HDMapUtil::BaseMapPtr();
  CHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

  // instantiate reference line provider
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(hdmap_);
  reference_line_provider_->Start();

  AERROR << "Initialized FakeObstacleComponent !";

  return true;
}

bool FakeObstacleComponent::InitReaders() {
  routing_reader_ = node_->CreateReader<RoutingResponse>(
      FLAGS_routing_response_topic,
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        AINFO << "Received routing data: run routing callback."
              << routing->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        latest_routing_.CopyFrom(*routing);
      });
  if (routing_reader_ == nullptr) {
    AERROR << "Create routing response reader failed";
    return false;
  }

  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis) {
        ADEBUG << "Received chassis data: run chassis callback."
               << chassis->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        latest_chassis_.CopyFrom(*chassis);
      });
  if (chassis_reader_ == nullptr) {
    AERROR << "Create chassis reader failed";
    return false;
  }

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<LocalizationEstimate>& localization) {
        AINFO << "Received localization data: run localization callback."
              << localization->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        latest_localization_.CopyFrom(*localization);
      });
  if (localization_reader_ == nullptr) {
    AERROR << "Create localization reader failed";
    return false;
  }

  return true;
}

bool FakeObstacleComponent::Proc() {
  FillPerceptionObstacles();
  return true;
}

bool FakeObstacleComponent::UpdateReferenceLine() {
  AINFO << "Get routing:" << latest_routing_.DebugString();
  if (!latest_routing_.has_header()) {
    AERROR << "routing not received yet";
    return false;
  }

  AINFO << "Get localization:" << latest_localization_.DebugString();
  if (!latest_localization_.has_header()) {
    AERROR << "localization not received yet";
    return false;
  }

  AINFO << "Get chassis:" << latest_chassis_.DebugString();
  if (!latest_chassis_.has_header()) {
    AERROR << "chassis not received yet";
    return false;
  }

  Status status = VehicleStateProvider::Instance()->Update(latest_localization_,
                                                           latest_chassis_);
  VehicleState vehicle_state =
      VehicleStateProvider::Instance()->vehicle_state();

  reference_line_provider_->UpdateRoutingResponse(latest_routing_);
  reference_line_provider_->UpdateVehicleState(vehicle_state);

  std::list<hdmap::RouteSegments> segments;
  if (!reference_line_provider_->GetReferenceLines(&last_reference_lines_,
                                                   &segments)) {
    AERROR << "Failed to create reference line";
    return false;
  }

  DCHECK_EQ(last_reference_lines_.size(), segments.size());

  return true;
}

bool FakeObstacleComponent::FillPerceptionObstacles() {
  if (!obstacle_has_started_) {
    if (!UpdateReferenceLine()) {
      AERROR << "Could not update reference";
      return false;
    }

    common::math::Vec2d ego_vec2d =
        common::math::Vec2d(latest_localization_.pose().position().x(),
                            latest_localization_.pose().position().y());
    common::SLPoint ego_sl;
    last_reference_lines_.front().XYToSL(ego_vec2d, &ego_sl);

    s_init_obstacle_ = ego_sl.s() + fake_obst_config_.delta_s_in_front();
    t_init_obstacle_ = Clock::NowInSeconds();
    obstacle_has_started_ = true;
  }

  double diff_time = Clock::NowInSeconds() - t_init_obstacle_;
  double s_current_obstacle =
      s_init_obstacle_ + diff_time * fake_obst_config_.velocity();
  planning::ReferencePoint other_ref_point =
      last_reference_lines_.front().GetReferencePoint(s_current_obstacle);

  auto response = std::make_shared<PerceptionObstacles>();
  // TODO: fill response
  apollo::perception::PerceptionObstacle* ob =
      response->add_perception_obstacle();

  apollo::common::Point3D position;
  position.set_x(other_ref_point.x());
  position.set_y(other_ref_point.y());
  position.set_z(0.0);
  ob->mutable_position()->CopyFrom(position);

  ob->set_theta(other_ref_point.heading());

  // Set linear_velocity
  apollo::common::Point3D velocity3d;
  velocity3d.set_x(std::cos(other_ref_point.heading()) *
                   fake_obst_config_.velocity());
  velocity3d.set_y(std::sin(other_ref_point.heading()) *
                   fake_obst_config_.velocity());
  velocity3d.set_z(0);

  ob->mutable_velocity()->CopyFrom(velocity3d);

  // Size of obstacle bounding box.
  ob->set_length(4);  // obstacle length.
  ob->set_width(2);   // obstacle width.
  ob->set_height(2);  // obstacle height.

  obstacle_writer_->Write(response);
  return true;
}

}  // namespace fake_obstacle
}  // namespace apollo
