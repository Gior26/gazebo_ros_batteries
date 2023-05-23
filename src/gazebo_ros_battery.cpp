/*
 * Based on https://github.com/nilseuropa/gazebo_ros_battery
 *	made by Marton Juhasz
 *
 * SPDX-FileCopyrightText: �2023 Giordano Scarso
 * SPDX-License-Identifier: GPL-3.0-or-later
*/
#include <assert.h>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include <sdf/sdf.hh>

#include <boost/make_shared.hpp>
#include <gazebo/transport/transport.hh>
#include "gazebo_ros_battery/gazebo_ros_battery.h"
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>


namespace gazebo {

class GazeboRosBatteryPrivate
{

  public:

      gazebo_ros::Node::SharedPtr ros_node_;
      event::ConnectionPtr update_connection_;
      physics::ModelPtr parent;
      rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_voltage_publisher_;
      rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_consumer_sub_;
      //std::vector<ros::Subscriber> current_draw_subscribers_;
      sensor_msgs::msg::BatteryState battery_state_;
      common::Time last_update_time_;
      std::string battery_topic_;
      std::string consumer_topic_;
      std::string battery_voltage_topic_;
      std::string frame_id_;
      std::string plugin_name_;
      std::string namespace_;
      std::vector<double> drawn_currents_;
      // common parameters
      bool publish_voltage_;
      int technology_;
      int num_of_consumers_;
      int cell_count_;
      double update_rate_;
      double update_period_;
      double design_capacity_;
      double current_drawn_;
      double nominal_voltage_;
      double constant_voltage_;
      double cut_off_voltage_;
      double internal_resistance_;
      double lpf_tau_;
      // linear model params
      double lin_discharge_coeff_;
      bool use_nonlinear_model_;
      // nonlinear model params
      double polarization_constant_; // polarization constant [V/Ah] or pol. resistance [Ohm]
      double exponential_voltage_;   // exponential voltage [V]
      double exponential_capacity_;  // exponential capacity [1/(Ah)]
      double characteristic_time_;   // characteristic time [s] for charge-dependence
      double design_temperature_;         // Design temperature where pol. const is unchanged and voltages are nominal.
      double arrhenius_rate_polarization_; // Arrhenius rate of polarization constant [K]
      double capacity_temp_coeff_;      // Temperature dependence of capacity [Ah/K]
      double reversible_voltage_temp_;  // Linear temperature dependant voltage shift [V/K]
      // internal variables
      bool model_initialised_;
      bool internal_cutt_off_;
      bool battery_empty_;
      double voltage_;
      double charge_;
      double charge_memory_;
      double current_;
      double discharge_;
      double capacity_;
      double temperature_;
      double temp_set_;
      double temp_lpf_tau_;
      double current_lpf_;

      // model update
      void linear_discharge_voltage_update();
      void nonlinear_discharge_voltage_update();

      // Services
      void SetCharge(const std::shared_ptr<battery_services::srv::SetCharge::Request> req,
		     std::shared_ptr<battery_services::srv::SetCharge::Response> res);

      void ResetModel(const std::shared_ptr<battery_services::srv::Reset::Request> req,
		      std::shared_ptr<battery_services::srv::Reset::Response> res);

      void SetTemperature(const std::shared_ptr<battery_services::srv::SetTemperature::Request> req,
			  std::shared_ptr<battery_services::srv::SetTemperature::Response> res);
      // Callback Queue
      //ros::CallbackQueue queue_;
      //std::thread callback_queue_thread_;
      //std::mutex lock_;
      //void QueueThread();
      void currentCallback(const std_msgs::msg::Float32::ConstPtr& current, int consumer_id);
      void OnUpdate();
      rclcpp::Service<battery_services::srv::SetTemperature>::SharedPtr set_temperature;
      rclcpp::Service<battery_services::srv::SetCharge>::SharedPtr set_charge_state;
      rclcpp::Service<battery_services::srv::Reset>::SharedPtr reset_model;

      std::mutex service_lock;
      void Reset();
};
// Constructor
GazeboRosBattery::GazeboRosBattery()
	: impl_(std::make_unique<GazeboRosBatteryPrivate>())
{
}

GazeboRosBattery::~GazeboRosBattery()
{
}

// Load the controller
void GazeboRosBattery::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
	impl_->parent = _parent;
	impl_->plugin_name_ = _sdf->GetAttribute("name")->GetAsString();
	impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
	const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

	// topic params
	impl_->num_of_consumers_ =_sdf->Get<int> ("num_of_consumers", 2 ).first;

	impl_->namespace_ = _sdf->Get<std::string> ("namespace", "" ).first;
	if (impl_->namespace_ != "")
		impl_->consumer_topic_ = impl_->namespace_ +"/"+ _sdf->Get<std::string> ("consumer_topic", "/battery/consumer" ).first;
	else
		impl_->consumer_topic_ = _sdf->Get<std::string> ("consumer_topic", "/battery/consumer" ).first;
	impl_->frame_id_ = _sdf->Get<std::string> ("frame_id", "battery" ).first;
	if (impl_->namespace_ != "")
		impl_->battery_topic_ = impl_->namespace_ +"/"+ _sdf->Get<std::string> ("battery_topic", "/battery/status" ).first;
	else
		impl_->battery_topic_ = _sdf->Get<std::string> ("battery_topic", "/battery/status" ).first;

	impl_->publish_voltage_ = _sdf->Get<bool> ("publish_voltage", true ).first;
	if (impl_->publish_voltage_){
		if (impl_->namespace_ != "")
			impl_->battery_voltage_topic_ = impl_->namespace_ +"/"+ _sdf->Get<std::string> ("battery_voltage_topic", "/battery/voltage" ).first;
		else
			impl_->battery_voltage_topic_ = _sdf->Get<std::string> ("battery_voltage_topic", "/battery/voltage" ).first;
	}

	// common parameters
	impl_->technology_ = _sdf->Get<int> ("technology", sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION ).first;
	impl_->cell_count_ = _sdf->Get<int> ("number_of_cells", 8 ).first;
	impl_->update_rate_ = _sdf->Get<double> ("update_rate", 10.0 ).first;
	impl_->design_capacity_ = _sdf->Get<double> ("design_capacity", 4.0 ).first;
	impl_->nominal_voltage_ = _sdf->Get<double> ("nominal_voltage", 24.0 ).first;
	impl_->cut_off_voltage_ = _sdf->Get<double> ("cut_off_voltage", 18.0 ).first;
	impl_->constant_voltage_ = _sdf->Get<double> ("full_charge_voltage", 24.2 ).first;
	impl_->lpf_tau_ = _sdf->Get<double> ("current_filter_tau", 1.0 ).first;
	impl_->temp_lpf_tau_ = _sdf->Get<double> ("temperature_response_tau", 0.5 ).first;
	impl_->internal_resistance_ = _sdf->Get<double> ("internal_resistance", 0.05 ).first;

	// model specific params
	impl_->use_nonlinear_model_ = _sdf->Get<bool> ("use_nonlinear_model", true ).first;
	if (!impl_->use_nonlinear_model_) {
		impl_->lin_discharge_coeff_ = _sdf->Get<double> ("linear_discharge_coeff", -1.0 ).first;
	}
	else {
		impl_->polarization_constant_ = _sdf->Get<double> ("polarization_constant", 0.07 ).first;
		impl_->exponential_voltage_ = _sdf->Get<double> ("exponential_voltage", 0.7 ).first;
		impl_->exponential_capacity_ = _sdf->Get<double> ("exponential_capacity", 3.0 ).first;
		impl_->characteristic_time_ = _sdf->Get<double> ("characteristic_time", 200.0 ).first;
		impl_->design_temperature_ = _sdf->Get<double> ("design_temperature", 25.0 ).first;
		impl_->arrhenius_rate_polarization_ = _sdf->Get<double> ("arrhenius_rate_polarization", 500.0 ).first;
		impl_->reversible_voltage_temp_ = _sdf->Get<double> ("reversible_voltage_temp", 0.05 ).first;
		impl_->capacity_temp_coeff_ = _sdf->Get<double> ("capacity_temp_coeff", 0.01 ).first;
	}

	impl_->battery_state_.header.frame_id = impl_->frame_id_;
	impl_->battery_state_.power_supply_technology = impl_->technology_;
	impl_->battery_state_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
	impl_->battery_state_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
	impl_->battery_state_.design_capacity = impl_->design_capacity_;
	impl_->battery_state_.capacity = impl_->design_capacity_;
	// battery_state_.capacity   = last_full_capacity_;
	impl_->battery_state_.present = true;
	impl_->battery_state_.temperature = impl_->temperature_ = impl_->design_temperature_;

	// start
	if ( impl_->update_rate_ > 0.0 )
		impl_->update_period_ = 1.0 / impl_->update_rate_;
	else
		impl_->update_period_ = 0.0;
	impl_->last_update_time_ = impl_->parent->GetWorld()->SimTime();
	// subscribers
//	impl_->battery_consumer_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(
//		impl_->consumer_topic_, qos.get_subscription_qos(impl_->consumer_topic_, rclcpp::QoS(1)),
//		std::bind(&GazeboRosBatteryPrivate::currentCallback, impl_.get(),
//			  std::placeholders::_1, std::placeholders::_2));

	//RCLCPP_INFO(
	//	    impl_->ros_node_->get_logger(), "Subscribed to [%s]",
	//	    impl_->battery_consumer_sub_->get_topic_name());

	// publishers
	impl_->battery_state_publisher_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::BatteryState>(
		impl_->battery_topic_, qos.get_publisher_qos(
		     impl_->battery_topic_, rclcpp::SensorDataQoS()
		     ));
	RCLCPP_INFO(
		    impl_->ros_node_->get_logger(), "Advertise battery_state on [%s]",
		    impl_->battery_state_publisher_->get_topic_name());
	if (impl_->publish_voltage_){
		impl_->battery_voltage_publisher_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(
			impl_->battery_voltage_topic_, qos.get_publisher_qos(
			     impl_->battery_voltage_topic_, rclcpp::SensorDataQoS()
			     ));
		RCLCPP_INFO(
			    impl_->ros_node_->get_logger(), "Advertise battery voltage topic on [%s]",
			    impl_->battery_voltage_publisher_->get_topic_name());
	}

	// start custom queue
	//this->callback_queue_thread_ = std::thread ( std::bind ( &GazeboRosBattery::QueueThread, this ) );

	// listen to the update event (broadcast every simulation iteration)
	impl_->update_connection_ = event::Events::ConnectWorldUpdateBegin(
		std::bind(&GazeboRosBatteryPrivate::OnUpdate, impl_.get()));

	// services
	impl_->set_charge_state = impl_->ros_node_->create_service<battery_services::srv::SetCharge>(
		impl_->plugin_name_ + "/set_charge", std::bind(
			&GazeboRosBatteryPrivate::SetCharge, impl_.get(),
			std::placeholders::_1,
			std::placeholders::_2));
	impl_->set_temperature = impl_->ros_node_->create_service<battery_services::srv::SetTemperature>(
		impl_->plugin_name_ + "/set_temperature", std::bind(
			&GazeboRosBatteryPrivate::SetTemperature, impl_.get(),
			std::placeholders::_1,
			std::placeholders::_2));
	impl_->reset_model = impl_->ros_node_->create_service<battery_services::srv::Reset>(
		impl_->plugin_name_ + "/reset", std::bind(
			&GazeboRosBatteryPrivate::ResetModel, impl_.get(),
			std::placeholders::_1,
			std::placeholders::_2));

	impl_->model_initialised_ = false;
	impl_->Reset();

}

void GazeboRosBatteryPrivate::SetTemperature(const std::shared_ptr<battery_services::srv::SetTemperature::Request> req,
					     std::shared_ptr<battery_services::srv::SetTemperature::Response> res)
{
	service_lock.lock();
	temp_set_ = req->temperature.data;
	//RCL_WARN_NAMED(plugin_name_, "%s: Temperature set: %f", gazebo_ros_->info(), temp_set_);
	service_lock.unlock();
	//return true; TODO Add exception on failure
}

void GazeboRosBatteryPrivate::SetCharge(const std::shared_ptr<battery_services::srv::SetCharge::Request> req,
					std::shared_ptr<battery_services::srv::SetCharge::Response> res)
{
	service_lock.lock();
	discharge_ = discharge_ + req->charge.data;
	//RCL_WARN_NAMED(plugin_name_, "%s: Charge set: %f", gazebo_ros_->info(), discharge_ + req->charge.data);
	service_lock.unlock();
	//return true TODO Add exception on failure;
}

void GazeboRosBatteryPrivate::ResetModel(const std::shared_ptr<battery_services::srv::Reset::Request> req,
					 std::shared_ptr<battery_services::srv::Reset::Response> res)
{
	service_lock.lock();
	Reset();
	service_lock.unlock();
	//return true //TODO Add exception on failure;
}

void GazeboRosBatteryPrivate::Reset() {
	last_update_time_ = parent->GetWorld()->SimTime();
	charge_ = design_capacity_;
	voltage_ = constant_voltage_;
	temperature_ = design_temperature_;
	temp_set_ = design_temperature_;
	discharge_ = 0;
	current_drawn_ = 1; //TODO Hardcoded value
	battery_empty_ = false;
	internal_cutt_off_ = false;
	model_initialised_ = true;
	RCLCPP_INFO(ros_node_->get_logger(), "Battery model reset.");
}

void GazeboRosBattery::Reset() {
	impl_->last_update_time_ = impl_->parent->GetWorld()->SimTime();
	impl_->charge_ = impl_->design_capacity_;
	impl_->voltage_ = impl_->constant_voltage_;
	impl_->temperature_ = impl_->design_temperature_;
	impl_->temp_set_ = impl_->design_temperature_;
	impl_->discharge_ = 0;
	impl_->current_drawn_ = 0;
	impl_->battery_empty_ = false;
	impl_->internal_cutt_off_ = false;
	impl_->model_initialised_ = true;
	RCLCPP_INFO(impl_->ros_node_->get_logger(), "Battery model reset.");
}

// simple linear discharge model
void GazeboRosBatteryPrivate::linear_discharge_voltage_update() {
	voltage_ = constant_voltage_ + lin_discharge_coeff_ * (1 - discharge_ / design_capacity_) - internal_resistance_ * current_lpf_;
}

// Temperature dependent parameters:
//    QT = Q + dQdT*(t-t0) //    KT = K * np.exp(Kalpha*(1/t-1/t0))
//    E0T = E0 + Tshift*(t-t0)
// v(i) = E0T - KT*QT/(QT-it)*(i*Ctime/3600.0+it) + A*np.exp(-B*it)
//
//  where
//  E0: constant voltage [V]
//  K:  polarization constant [V/Ah] or pol. resistance [Ohm]
//  Q:  maximum capacity [Ah]
//  A:  exponential voltage [V]
//  B:  exponential capacity [A/h]
//  it: already extracted capacity [Ah] (= - discharge)
//  id: current * characteristic time [Ah]
//  i:  battery current [A]
//  T0: design temperature [Celsius]
// Tpol: name of the Vulkan first officer aboard Enterprise NX-01 [Celsius]
// Tshift: temperature offset [Celsius]

void GazeboRosBatteryPrivate::nonlinear_discharge_voltage_update() {
	double t = temperature_+273.15;
	double t0 = design_temperature_+273.15;
	double E0T = constant_voltage_ + reversible_voltage_temp_*(t-t0); // TODO: Don't increase for t>t0 ?
	double QT = design_capacity_ + capacity_temp_coeff_*(t-t0);       // TODO: Don't increase for t>t0 ?
	double KT = polarization_constant_ * exp(arrhenius_rate_polarization_*(1/t-1/t0));
	voltage_ = E0T
		- KT * QT/(QT + discharge_) * (current_lpf_*(characteristic_time_/3600.0) - discharge_)
		+ exponential_voltage_*exp(-exponential_capacity_*-discharge_);
}

// Plugin update function
void GazeboRosBatteryPrivate::OnUpdate()
{
	common::Time current_time = parent->GetWorld()->SimTime();
	double dt = ( current_time - last_update_time_ ).Double();

	if ( dt > update_period_ && model_initialised_ ) {

		double n = dt / temp_lpf_tau_;
		temperature_ = temperature_ + n * (temp_set_ - temperature_);

		if (!battery_empty_) {
			// LPF filter on current
			double k = dt / lpf_tau_;
			current_lpf_ = current_lpf_ + k * (current_drawn_ - current_lpf_);
			// Accumulate discharge (negative: 0 to design capacity)
			discharge_ = discharge_ - GZ_SEC_TO_HOUR(dt * current_lpf_);
			if (!internal_cutt_off_) {
				if (use_nonlinear_model_){
					nonlinear_discharge_voltage_update();
				}
				else {
					linear_discharge_voltage_update();
				}
				charge_ = design_capacity_ + discharge_; // discharge is negative
				charge_memory_ = charge_;
			}
			if (voltage_<=cut_off_voltage_ && !internal_cutt_off_) {
				discharge_ = 0;
				voltage_ = 0;
				internal_cutt_off_ = true;
				charge_ = charge_memory_;
				//RCL_WARN_NAMED(plugin_name_, "%s: Battery voltage cut off.", gazebo_ros_->info());
			}
		}

		if (!battery_empty_ && charge_<=0) {
			discharge_ = 0;
			current_lpf_ = 0;
			voltage_ = 0;
			battery_empty_ = true;
			//RCL_WARN_NAMED(plugin_name_, "%s: Battery discharged.", gazebo_ros_->info());
		}
		// state update
		battery_state_.header.stamp = this->ros_node_->now();
		battery_state_.voltage    = voltage_;
		battery_state_.current    = current_lpf_;
		battery_state_.charge     = charge_;
		battery_state_.percentage = (charge_/design_capacity_)*100;
		battery_state_.temperature = temperature_;
		battery_state_publisher_->publish( battery_state_ );
		std_msgs::msg::Float32 battery_voltage_;
		battery_voltage_.data = voltage_;
		if (publish_voltage_) battery_voltage_publisher_->publish( battery_voltage_ );
		last_update_time_+= common::Time ( update_period_ );
	}
}

void GazeboRosBatteryPrivate::currentCallback(const std_msgs::msg::Float32::ConstPtr& current, int consumer_id) {

	current_drawn_ = 0;
	drawn_currents_[consumer_id]=current->data;
	for (int i=0; i<num_of_consumers_; i++) current_drawn_+=drawn_currents_[i];
	// RCL_WARN("Total current drawn: %f", current_drawn_);

	// temporary solution for simple charging
	if (current_drawn_ <=0){
		battery_state_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
		if (internal_cutt_off_) voltage_ = cut_off_voltage_ + 0.05;
		internal_cutt_off_ = false;
	}
	else {
		battery_state_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

//void GazeboRosBattery::QueueThread() {
//	static const double timeout = 0.01;
//	while ( rclcpp::ok() ) {
//		queue_.callAvailable ( ros::WallDuration ( timeout ) );
//	}
//}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosBattery )
	// eof_ns
	}
