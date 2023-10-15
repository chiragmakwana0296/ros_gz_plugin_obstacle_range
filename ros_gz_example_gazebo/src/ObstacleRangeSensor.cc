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
#include <ros_gz_example_gazebo/msgs/obstaclerange.pb.h>
#include "ros_gz_example_gazebo/ObstacleRangeSensor.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    ros_gz_example_gazebo::ObstacleRangeSensor,
    gz::sim::System,
    ros_gz_example_gazebo::ObstacleRangeSensor::ISystemConfigure,
    ros_gz_example_gazebo::ObstacleRangeSensor::ISystemPreUpdate,
    ros_gz_example_gazebo::ObstacleRangeSensor::ISystemUpdate,
    ros_gz_example_gazebo::ObstacleRangeSensor::ISystemPostUpdate
    // ros_gz_example_gazebo::ObstacleRangeSensor::ISystemReset
)

namespace ros_gz_example_gazebo 
{

void ObstacleRangeSensor::Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager)
{
  pub = this->node.Advertise<gz::msgs::LaserScan>("/range_bearing");
  // gzerr << "ros_gz_example_gazebo::ObstacleRangeSensor::Configure on entity: " << _entity << std::endl;
}

void ObstacleRangeSensor::PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused)
  {
    // gzerr << "ros_gz_example_gazebo::ObstacleRangeSensor::PreUpdate" << std::endl;
    _ecm.EachNew<gz::sim::components::GpuLidar,
               gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::GpuLidar *_sensor,
        const gz::sim::components::ParentEntity *_parent)->bool
      {
        auto sensorScopedName = gz::sim::removeParentScope(
            gz::sim::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _sensor->Data();
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

        auto parentName = _ecm.Component<gz::sim::components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        _ecm.CreateComponent(_entity,
            gz::sim::components::SensorTopic(sensor->Topic()));

        this->entitySensorMap.insert(std::make_pair(_entity,
            std::move(sensor)));

        gzdbg << data.Topic() << std::endl;
        return true;
      });

  }
}


void ObstacleRangeSensor::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused )
  {
    // gzerr << "ros_gz_example_gazebo::ObstacleRangeSensor::Update" << std::endl;
    for (auto &[entity, sensor] : this->entitySensorMap)
    {
      sensor->Update(_info.simTime);
      ros_gz_example_gazebo::msgs::ObstacleRange msg;
      msg.clear_range();
      msg.clear_bearing();
      for(int index = 0; index<=sensor->RangeCount(); index++){
        msg.add_range(sensor->Range(index));
        msg.add_bearing(index * sensor->AngleResolution());
        
        this->pub.Publish(msg);

      }
      gzdbg << sensor->AngleMin() << std::endl;
    }
  }
}

void ObstacleRangeSensor::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) 
{
  if (!_info.paused)
  {
  //   for (auto &[entity, sensor] : this->entitySensorMap)
  //   {
  //     // ros_gz_example_gazebo::msgs::FoobarStamped msg;
  //     // msg.clear_range();

  //     gzdbg << sensor->AngleMin() << std::endl;
  //   }
  this->RemoveSensorEntities(_ecm);

  }

  // gzerr << "ros_gz_example_gazebo::ObstacleRangeSensor::PostUpdate" << std::endl;
}


void ObstacleRangeSensor::RemoveSensorEntities(
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

void ObstacleRangeSensor::Reset(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm)
{
  gzerr << "ros_gz_example_gazebo::ObstacleRangeSensor::Reset" << std::endl;
}
}  // namespace ros_gz_example_gazeba
