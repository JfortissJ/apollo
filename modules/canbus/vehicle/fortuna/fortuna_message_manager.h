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

/**
 * @file fortuna_message_manager.h
 * @brief the class of FortunaMessageManager
 */

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

/**
 * @namespace apollo::canbus::fortuna
 * @brief apollo::canbus::fortuna
 */
namespace apollo {
namespace canbus {
namespace fortuna {

using ::apollo::drivers::canbus::MessageManager;

/**
 * @class FortunaMessageManager
 *
 * @brief implementation of MessageManager for fortuna vehicle
 */
class FortunaMessageManager
    : public MessageManager<::apollo::canbus::ChassisDetail> {
 public:
  /**
   * @brief construct a fortuna message manager. protocol data for send and
   * receive are added in the contruction.
   */
  FortunaMessageManager();
  virtual ~FortunaMessageManager();
};

}  // namespace fortuna
}  // namespace canbus
}  // namespace apollo
