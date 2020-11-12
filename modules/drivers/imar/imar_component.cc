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

#include "modules/drivers/imar/imar_component.h"

#include "modules/drivers/imar/imar_ethernet.h"

namespace apollo {
namespace drivers {
namespace imar {

using apollo::cyber::proto::RoleAttributes;

imarDriverComponent::imarDriverComponent()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::
              GNSS)  //! todo introduce in monitor logger protobuf, for the
                     //! moment take the gnss entry
{
  AERROR << "Started imar node!";
}

bool imarDriverComponent::Init() {
  imarConf imar_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &imar_config)) {
    monitor_logger_buffer_.ERROR("Unable to load imar conf file: " +
                                 config_file_path_);
    return false;
  }

  imar_ethernet_.reset(new ImarEthernet(node_, imar_config));

  if (!imar_ethernet_->Init()) {
    monitor_logger_buffer_.ERROR("Init gnss stream failed");
    AERROR << "Init gnss stream failed";
    return false;
  }
  imar_ethernet_->Start();

  monitor_logger_buffer_.INFO("gnss is (not yet) started.");
  return true;
}

}  // namespace imar
}  // namespace drivers
}  // namespace apollo
