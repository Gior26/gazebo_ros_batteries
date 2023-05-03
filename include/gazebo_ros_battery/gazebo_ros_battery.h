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
//#include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>

// Custom Callback Queue
//#include <ros/callback_queue.h>
//#include <ros/advertise_options.h>

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
