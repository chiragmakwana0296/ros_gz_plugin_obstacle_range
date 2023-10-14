/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
 *
*/

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <unordered_map>
#include <utility>

#include <gz/common/Profiler.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/GpuLidarSensor.hh>
#include <sdf/Sensor.hh>

#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Lidar.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

// Don't forget to include the plugin's header.
#include "ros_gz_example_gazebo/FullSystem.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    ros_gz_example_gazebo::FullSystem,
    gz::sim::System,
    // ros_gz_example_gazebo::FullSystem::ISystemConfigure,
    ros_gz_example_gazebo::FullSystem::ISystemPreUpdate,
    // ros_gz_example_gazebo::FullSystem::ISystemUpdate,
    ros_gz_example_gazebo::FullSystem::ISystemPostUpdate
    // ros_gz_example_gazebo::FullSystem::ISystemReset
)

namespace ros_gz_example_gazebo 
{

void FullSystem::Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager)
{
  // gzerr << "ros_gz_example_gazebo::FullSystem::Configure on entity: " << _entity << std::endl;
}

void FullSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused)
  {
    // gzerr << "ros_gz_example_gazebo::FullSystem::PreUpdate" << std::endl;
    _ecm.EachNew<gz::sim::components::GpuLidar,
               gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::GpuLidar *_sensor,
        const gz::sim::components::ParentEntity *_parent)->bool
      {
        auto sensorScopedName = gz::sim::removeParentScope(
            gz::sim::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _sensor->Data();
        // gzerr << data << std::endl;
        data.SetName(sensorScopedName);

        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/range";
          data.SetTopic(topic);
        }
        gz::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<gz::sensors::GpuLidarSensor>(data);
        if (nullptr == sensor)
        {
          gzerr << "Failed to create range [" << sensorScopedName << "]"
                 << std::endl;
          return false;
        }

        // Set sensor parent
        auto parentName = _ecm.Component<gz::sim::components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic on Gazebo
        _ecm.CreateComponent(_entity,
            gz::sim::components::SensorTopic(sensor->Topic()));

        // Keep track of this sensor
        this->entitySensorMap.insert(std::make_pair(_entity,
            std::move(sensor)));

        gzdbg << data.Topic() << std::endl;
        // gzerr << "ros_gz_example_gazebo::FullSystem::PreUpdate" << std::endl;
        return true;
      });

  }
}


void FullSystem::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzerr << "ros_gz_example_gazebo::FullSystem::Update" << std::endl;
  }
}

void FullSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) 
{
  if (!_info.paused)
  {
    for (auto &[entity, sensor] : this->entitySensorMap)
    {
      gzerr << sensor->AngleMin() << std::endl;;
    }
  }
  this->RemoveSensorEntities(_ecm);

  // gzerr << "ros_gz_example_gazebo::FullSystem::PostUpdate" << std::endl;
}


void FullSystem::RemoveSensorEntities(
    const gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<gz::sim::components::GpuLidar>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::GpuLidar *)->bool
      {
        if (this->entitySensorMap.erase(_entity) == 0)
        {
          gzerr << "Internal error, missing odometer for entity ["
                         << _entity << "]" << std::endl;
        }
        return true;
      });
}

void FullSystem::Reset(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm)
{
  gzerr << "ros_gz_example_gazebo::FullSystem::Reset" << std::endl;
}
}  // namespace ros_gz_example_gazeba
