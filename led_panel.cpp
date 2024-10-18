#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/msg/led_state_array.hpp"
#include "my_interfaces/srv/set_led.hpp"

using namespace std::chrono_literals;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode()
    : Node("led_panel")
    {
        led_states_ = {0, 0, 0};
        led_states_publisher_ = this->create_publisher<my_interfaces::msg::LedStateArray>("led_states", 10);
        led_states_timer_ = this->create_wall_timer(4s, std::bind(&LedPanelNode::publish_led_states, this));
        set_led_service_ = this->create_service<my_interfaces::srv::SetLed>(
            "set_led",
            std::bind(&LedPanelNode::callback_set_led, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "Led panel node has been started.");
    }

private:
    void publish_led_states()
    {
        auto msg = my_interfaces::msg::LedStateArray();
        msg.led_states = led_states_;
        led_states_publisher_->publish(msg);
    }

    void callback_set_led(
        const std::shared_ptr<my_interfaces::srv::SetLed::Request> request,
        std::shared_ptr<my_interfaces::srv::SetLed::Response> response)
    {
        int64_t led_num = request->led_number;
        int64_t state = request->state;

        if (led_num > static_cast<int64_t>(led_states_.size()) || led_num <= 0)
        {
            response->success = false;
            return;
        }

        if (state != 0 && state != 1)
        {
            response->success = false;
            return;
        }

        led_states_[led_num - 1] = state;
        response->success = true;
        publish_led_states();
    }

    std::vector<int64_t> led_states_;
    rclcpp::Publisher<my_interfaces::msg::LedStateArray>::SharedPtr led_states_publisher_;
    rclcpp::TimerBase::SharedPtr led_states_timer_;
    rclcpp::Service<my_interfaces::srv::SetLed>::SharedPtr set_led_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
