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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/drivers/imar/imar_ethernet.h"

namespace apollo {
namespace drivers {
namespace imar {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::imar::ImarEthernet;

class imarDriverComponent : public Component<> {
 public:
  imarDriverComponent();
  bool Init() override;

 private:
  std::unique_ptr<ImarEthernet> imar_ethernet_ = nullptr;
  apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

CYBER_REGISTER_COMPONENT(imarDriverComponent)

}  // namespace imar
}  // namespace drivers
}  // namespace apollo
