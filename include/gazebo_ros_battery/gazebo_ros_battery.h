/*
 * Based on https://github.com/nilseuropa/gazebo_ros_battery
 *	made by Marton Juhasz
 *
 * SPDX-FileCopyrightText: �2023 Giordano Scarso
 * SPDX-License-Identifier: GPL-3.0-or-later
*/
#ifndef _MOTOR_PLUGIN_H_
#define _MOTOR_PLUGIN_H_

#include <map>
#include <memory>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions/sensor_msgs.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>

// Boost
#include <boost/thread.hpp>

// Services
#include <battery_services/srv/set_temperature.hpp>
#include <battery_services/srv/set_charge.hpp>
#include <battery_services/srv/reset.hpp>

namespace gazebo {

  class Joint;
  class Entity;
  class GazeboRosBatteryPrivate;

  class GazeboRosBattery : public ModelPlugin {

    public:

      GazeboRosBattery();
      ~GazeboRosBattery();

    protected:

      void Load(physics::ModelPtr  _parent, sdf::ElementPtr _sdf) override;
      void Reset() override;

    private:
      std::unique_ptr<GazeboRosBatteryPrivate> impl_;
  };

}

#endif
