#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/srv/set_led.hpp"

using namespace std::chrono_literals;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode()
    : Node("battery"),
      battery_state_("F"),
      last_time_battery_state_changed_(this->now())
    {
        battery_timer_ = this->create_wall_timer(100ms, std::bind(&BatteryNode::check_battery_state, this));
        client_ = this->create_client<my_interfaces::srv::SetLed>("set_led");
        RCLCPP_INFO(this->get_logger(), "Battery checker is now on");
    }

private:
    void check_battery_state()
    {
        auto time_now = this->now();
        auto duration = time_now - last_time_battery_state_changed_;
        if (battery_state_ == "F")
        {
            if (duration.seconds() > 4.0)
            {
                battery_state_ = "E";
                RCLCPP_INFO(this->get_logger(), "Battery is empty! Charging battery...");
                last_time_battery_state_changed_ = time_now;
                call_set_led_server(3, 1);
            }
        }
        else
        {
            if (duration.seconds() > 6.0)
            {
                battery_state_ = "F";
                RCLCPP_INFO(this->get_logger(), "Battery finally full!");
                last_time_battery_state_changed_ = time_now;
                call_set_led_server(3, 0);
            }
        }
    }

    void call_set_led_server(int64_t led_number, int64_t state)
    {
        if (!client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Set Led Server...");
            return;
        }

        auto request = std::make_shared<my_interfaces::srv::SetLed::Request>();
        request->led_number = led_number;
        request->state = state;

        auto future = client_->async_send_request(
            request,
            std::bind(&BatteryNode::callback_call_set_led, this, std::placeholders::_1)
        );
    }

    void callback_call_set_led(rclcpp::Client<my_interfaces::srv::SetLed>::SharedFuture future)
    {
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "has the status of the LEDs been updated? -> %s", response->success ? "Sí" : "No");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    std::string battery_state_;
    rclcpp::Time last_time_battery_state_changed_;
    rclcpp::TimerBase::SharedPtr battery_timer_;
    rclcpp::Client<my_interfaces::srv::SetLed>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
