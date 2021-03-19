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
#include "modules/fake_obstacle/proto/fake_obstacle_conf.pb.h"

namespace apollo {
namespace fake_obstacle {

// using apollo::hdmap::HDMapUtil;
using apollo::canbus::Chassis;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::hdmap::HDMapUtil;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ReferenceLineProvider;
using apollo::routing::RoutingResponse;

FakeObstacleComponent::FakeObstacleComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {
  AERROR << "Started fake obstacle node!";
}

FakeObstacleComponent::~FakeObstacleComponent() {
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
}

bool FakeObstacleComponent::Init() {
  // load proto file
  FakeObstacleConf fake_obst_config;
  if (!cyber::common::GetProtoFromFile(config_file_path_, &fake_obst_config)) {
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
  CHECK(hdmap_) << "Failed to load map";

  // instantiate reference line provider
  // reference_line_provider_ = std::make_unique<ReferenceLineProvider>(hdmap_);
  // // does not build, maybe some compiler version issue?
  reference_line_provider_ =
      std::unique_ptr<ReferenceLineProvider>(new ReferenceLineProvider(hdmap_));
  reference_line_provider_->Start();

  return true;
}

bool FakeObstacleComponent::InitReaders() {
  // routing_reader_ = node_->CreateReader<RoutingResponse>(
  //     FLAGS_routing_response_topic,
  //     [this](const std::shared_ptr<RoutingResponse>& routing) {
  //       AINFO << "Received routing data: run routing callback."
  //             << routing->header().DebugString();
  //       std::lock_guard<std::mutex> lock(mutex_);
  //       latest_routing_.CopyFrom(*routing);
  //     });

  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis) {
        ADEBUG << "Received chassis data: run chassis callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        latest_chassis_.CopyFrom(*chassis);
      });

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<LocalizationEstimate>& localization) {
        AINFO << "Received localization data: run localization callback."
              << localization->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        latest_localization_.CopyFrom(*localization);
      });

  return true;
}

bool FakeObstacleComponent::Proc(
    const std::shared_ptr<RoutingResponse>& routing) {
  bool success = UpdateReferenceLine(routing);

  auto response = std::make_shared<PerceptionObstacles>();
  // TODO: fill response
  apollo::perception::PerceptionObstacle* ob =
      response->add_perception_obstacle();

  apollo::common::Point3D position;
  position.set_x(latest_localization_.pose().position().x() + 5);
  position.set_y(latest_localization_.pose().position().y() + 5);
  position.set_z(latest_localization_.pose().position().z());
  ob->mutable_position()->CopyFrom(position);

  // optional double theta = 3;  // heading in the world coordinate system.
  ob->set_theta(latest_localization_.pose().heading());

  // TODO: fill velocity
  // optional apollo.common.Point3D velocity = 4;  // obstacle velocity.

  // Size of obstacle bounding box.
  ob->set_length(4);  // obstacle length.
  ob->set_width(2);   // obstacle width.
  ob->set_height(2);  // obstacle height.

  obstacle_writer_->Write(response);

  return true;
}

bool FakeObstacleComponent::UpdateReferenceLine(
    const std::shared_ptr<routing::RoutingResponse>& routing) {
  ADEBUG << "Get localization:" << latest_localization_.DebugString();
  ADEBUG << "Get chassis:" << latest_chassis_.DebugString();

  Status status = VehicleStateProvider::Instance()->Update(latest_localization_,
                                                           latest_chassis_);
  VehicleState vehicle_state =
      VehicleStateProvider::Instance()->vehicle_state();

  reference_line_provider_->UpdateRoutingResponse(*routing);
  reference_line_provider_->UpdateVehicleState(vehicle_state);

  std::list<hdmap::RouteSegments> segments;
  if (!reference_line_provider_->GetReferenceLines(&last_reference_lines_,
                                                   &segments)) {
    AERROR << "Failed to create reference line";
    return false;
  }
  return true;
}

}  // namespace fake_obstacle
}  // namespace apollo
