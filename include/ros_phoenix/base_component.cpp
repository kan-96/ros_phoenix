#include "ros_phoenix/base_component.hpp"

#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "rclcpp/qos.hpp"
#include "rcutils/logging_macros.h"

#include <chrono>

using namespace rclcpp;
using namespace std::chrono_literals;

namespace ros_phoenix
{

    template <class MotorController, class Configuration, class FeedbackDevice, class ControlMode>
    BaseComponent<MotorController, Configuration, FeedbackDevice, ControlMode>::BaseComponent(const NodeOptions &options)
        : Node("talon", options)
    {
        this->declare_parameter<std::string>("interface", "can0");
        ctre::phoenix::platform::can::SetCANInterface(this->get_parameter("interface").as_string().c_str());

        this->declare_parameter<int>("id", 0);
        this->declare_parameter<int>("period_ms", 20);
        this->declare_parameter<int>("follow", -1);
        this->declare_parameter<int>("edges_per_rot", 4096); // Encoder edges per rotation (4096 is for built-in encoder)
        this->declare_parameter<bool>("invert", false);
        this->declare_parameter<bool>("invert_sensor", false);
        this->declare_parameter<bool>("brake_mode", true);
        this->declare_parameter<bool>("analog_input", false);
        this->declare_parameter<double>("max_voltage", 12);
        this->declare_parameter<double>("max_current", 30);

        this->declare_parameter<double>("P", 0);
        this->declare_parameter<double>("I", 0);
        this->declare_parameter<double>("D", 0);
        this->declare_parameter<double>("F", 0);

        this->controller_ = std::make_shared<MotorController>(this->get_parameter("id").as_int());
        this->timer_ = timer_ = this->create_wall_timer(
            std::chrono::milliseconds(this->get_parameter("period_ms").as_int()),
            std::bind(&BaseComponent::onTimer, this));

        this->set_on_parameters_set_callback(std::bind(&BaseComponent::reconfigure, this, std::placeholders::_1));
        this->reconfigure({});

        std::string name(this->get_name());

        this->pub_ = this->create_publisher<ros_phoenix::msg::MotorStatus>(name + "/status", 1);

        this->sub_ = this->create_subscription<ros_phoenix::msg::MotorControl>(name + "/set", 1,
                                                                               std::bind(&BaseComponent::set, this, std::placeholders::_1));
    }

    template <class MotorController, class Configuration, class FeedbackDevice, class ControlMode>
    BaseComponent<MotorController, Configuration, FeedbackDevice, ControlMode>::~BaseComponent()
    {
        std::lock_guard<std::mutex> guard(this->config_mutex_);
        if (this->config_thread_)
        {
            this->configured_ = true; // Signal config thread to stop
            this->config_thread_->join();
        }
    }

    template <class MotorController, class Configuration, class FeedbackDevice, class ControlMode>
    rcl_interfaces::msg::SetParametersResult
    BaseComponent<MotorController, Configuration, FeedbackDevice, ControlMode>::reconfigure(const std::vector<Parameter> &params)
    {
        std::lock_guard<std::mutex> guard(this->config_mutex_);

        for (auto param : params)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Parameter changed: %s=%s",
                        param.get_name().c_str(),
                        param.value_to_string().c_str());
            if (param.get_name() == "id" || !this->controller_)
            {
                this->controller_.reset();
                this->controller_ = std::make_shared<MotorController>(param.as_int());
            }
            else if (param.get_name() == "period_ms" || !this->timer_)
            {
                timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(param.as_int()),
                    std::bind(&BaseComponent::onTimer, this));
            }
        }

        if (!this->config_thread_)
        {
            this->configured_ = false;
            this->config_thread_ = std::make_shared<std::thread>(std::bind(&BaseComponent::configure, this));
        }
        else if (this->configured_)
        { // Thread needs to be joined and restarted
            this->config_thread_->join();
            this->configured_ = false;
            this->config_thread_ = std::make_shared<std::thread>(std::bind(&BaseComponent::configure, this));
        }

        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        return result;
    }

    template <class MotorController, class Configuration, class FeedbackDevice, class ControlMode>
    void BaseComponent<MotorController, Configuration, FeedbackDevice, ControlMode>::configure()
    {
        bool warned = false;
        while (!this->configured_)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::lock_guard<std::mutex> guard(this->config_mutex_);
            if (this->configured_)
                break; // Break out if signaled to stop while sleeping

            if (this->controller_->GetFirmwareVersion() == -1)
            {
                if (!warned)
                {
                    RCLCPP_WARN(this->get_logger(), "Motor controller has not been seen and can not be configured!");
                    warned = true;
                }
                continue;
            }
            SlotConfiguration slot;
            slot.kP = this->get_parameter("P").as_double();
            slot.kI = this->get_parameter("I").as_double();
            slot.kD = this->get_parameter("D").as_double();
            slot.kF = this->get_parameter("F").as_double();

            Configuration config;
            config.slot0 = slot;
            config.voltageCompSaturation = this->get_parameter("max_voltage").as_double();
            config.pulseWidthPeriod_EdgesPerRot = this->get_parameter("edges_per_rot").as_int();
            this->configure_current_limit(config);

            ErrorCode error = this->controller_->ConfigAllSettings(config, 50); // Takes up to 50ms
            if (error != ErrorCode::OK)
            {
                if (!warned)
                {
                    RCLCPP_WARN(this->get_logger(), "Motor controller configuration failed!");
                    warned = true;
                }
                continue;
            }

            if (this->get_parameter("brake_mode").as_bool())
                this->controller_->SetNeutralMode(NeutralMode::Brake);
            else
                this->controller_->SetNeutralMode(NeutralMode::Coast);

            this->configure_sensor();

            this->controller_->EnableVoltageCompensation(true);
            this->controller_->SetInverted(this->get_parameter("inverted").as_bool());
            this->controller_->SetSensorPhase(this->get_parameter("invert_sensor").as_bool());
            this->controller_->SelectProfileSlot(0, 0);

            int follow = this->get_parameter("follow").as_int();
            if (follow > 0)
                this->controller_->Set(ControlMode::Follower, follow);

            RCLCPP_INFO(this->get_logger(), "Successfully configured Motor Controller");
            this->configured_ = true;
        }
    }

    template <class MotorController, class Configuration, class FeedbackDevice, class ControlMode>
    void BaseComponent<MotorController, Configuration, FeedbackDevice, ControlMode>::configure_current_limit(Configuration &config)
    {
    }

    template <class MotorController, class Configuration, class FeedbackDevice, class ControlMode>
    void BaseComponent<MotorController, Configuration, FeedbackDevice, ControlMode>::configure_sensor()
    {
    }

    template <class MotorController, class Configuration, class FeedbackDevice, class ControlMode>
    double BaseComponent<MotorController, Configuration, FeedbackDevice, ControlMode>::get_output_current()
    {
        return 0; // Return 0 for devices which don't support current monitoring
    }

    template <class MotorController, class Configuration, class FeedbackDevice, class ControlMode>
    void BaseComponent<MotorController, Configuration, FeedbackDevice, ControlMode>::onTimer()
    {
        ctre::phoenix::unmanaged::FeedEnable(2 * this->get_parameter("period_ms").as_int());
        if (!this->configured_)
            return;

        ros_phoenix::msg::MotorStatus status;

        status.temperature = this->controller_->GetTemperature();
        status.bus_voltage = this->controller_->GetBusVoltage();

        status.output_percent = this->controller_->GetMotorOutputPercent();
        status.output_voltage = this->controller_->GetMotorOutputVoltage();
        status.output_current = this->get_output_current();

        status.position = this->controller_->GetSelectedSensorPosition();
        status.velocity = this->controller_->GetSelectedSensorVelocity();

        this->pub_->publish(status);
    }

    template <class MotorController, class Configuration, class FeedbackDevice, class ControlMode>
    void BaseComponent<MotorController, Configuration, FeedbackDevice, ControlMode>::set(ros_phoenix::msg::MotorControl::UniquePtr msg)
    {
        if (this->configured_)
            this->controller_->Set(static_cast<ControlMode>(msg->mode), msg->value);
    }

} // namespace ros_phoenix