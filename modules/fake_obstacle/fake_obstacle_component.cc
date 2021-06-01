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
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace fake_obstacle {

// using apollo::hdmap::HDMapUtil;
using apollo::canbus::Chassis;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ReferenceLineProvider;
using apollo::routing::RoutingResponse;

FakeObstacleComponent::FakeObstacleComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {
  AINFO << "Started FakeObstacleComponent !";
}

FakeObstacleComponent::~FakeObstacleComponent() {
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
}

bool FakeObstacleComponent::Init() {
  // load proto file
  if (!cyber::common::GetProtoFromFile(config_file_path_,
                                       &fake_obst_list_config_)) {
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
  auto obstacles = FillPerceptionObstacles();
  auto response = std::make_shared<PerceptionObstacles>(obstacles);
  common::util::FillHeader(node_->Name(), response.get());

  obstacle_writer_->Write(response);

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

PerceptionObstacles FakeObstacleComponent::FillPerceptionObstacles() {
  PerceptionObstacles obstacles;

  for (const auto& fake_obst_config :
       fake_obst_list_config_.fake_obstacle_conf()) {
    int id = fake_obst_config.id();
    if (obstacles_init_states_.count(id) == 0) {
      AERROR << "Initializing fake obstacle with id " << id;
      if (!UpdateReferenceLine()) {
        AERROR << "Could not update reference";
        break;
      }

      common::math::Vec2d ego_vec2d =
          common::math::Vec2d(latest_localization_.pose().position().x(),
                              latest_localization_.pose().position().y());
      common::SLPoint ego_sl;
      last_reference_lines_.front().XYToSL(ego_vec2d, &ego_sl);

      ObstacleInitState obstacle_init_state;
      obstacle_init_state.s = ego_sl.s() + fake_obst_config.delta_s_in_front();
      obstacle_init_state.t = Clock::NowInSeconds();
      obstacles_init_states_.insert(std::make_pair(id, obstacle_init_state));
    }

    double diff_time = Clock::NowInSeconds() - obstacles_init_states_[id].t;
    double s_current_obstacle =
        obstacles_init_states_[id].s + diff_time * fake_obst_config.velocity();
    planning::ReferencePoint other_ref_point =
        last_reference_lines_.front().GetReferencePoint(s_current_obstacle);

    auto* ob = obstacles.add_perception_obstacle();

    apollo::common::Point3D position;
    const double lateral_offset_direction = other_ref_point.heading() + M_PI_2;
    position.set_x(other_ref_point.x() + cos(lateral_offset_direction) *
                                             fake_obst_config.lateral_offset());
    position.set_y(other_ref_point.y() + sin(lateral_offset_direction) *
                                             fake_obst_config.lateral_offset());
    position.set_z(0.0);
    ob->mutable_position()->CopyFrom(position);

    ob->set_theta(other_ref_point.heading() +
                  fake_obst_config.heading_offset());

    // Set linear_velocity
    apollo::common::Point3D velocity3d;
    // TODO: why not use ob->theta??
    velocity3d.set_x(std::cos(other_ref_point.heading()) *
                     fake_obst_config.velocity());
    velocity3d.set_y(std::sin(other_ref_point.heading()) *
                     fake_obst_config.velocity());
    velocity3d.set_z(0);

    ob->mutable_velocity()->CopyFrom(velocity3d);

    // Size of obstacle bounding box.
    ob->set_length(fake_obst_config.length());
    ob->set_width(fake_obst_config.width());
    ob->set_height(fake_obst_config.height());

    ob->set_id(id);
    ob->set_type(fake_obst_config.type());
    ob->set_tracking_time(
        0.0);  // duration of an obstacle since detection in s.
    ob->set_timestamp(Clock::NowInSeconds());

    // obstacle corner points.
    Vec2d vec_obstacle = Vec2d(position.x(), position.y());
    Box2d obstacle_box =
        Box2d(vec_obstacle, ob->theta(), ob->length(), ob->width());
    std::vector<common::math::Vec2d> corner_points;
    obstacle_box.GetAllCorners(&corner_points);
    for (const auto& corner_point : corner_points) {
      auto* point = ob->add_polygon_point();
      point->set_x(corner_point.x());
      point->set_y(corner_point.y());
    }
  }
  return obstacles;
}

}  // namespace fake_obstacle
}  // namespace apollo
