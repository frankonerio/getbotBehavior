#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class BatteryPub : public rclcpp::Node
{
  public:
    BatteryPub()
    : Node("battery_pub"), count_(0)
    {
      Battery_Pub = this->create_publisher<std_msgs::msg::Int32>("/battery_pub", 1);
      timer_ = this->create_wall_timer(
      5s, std::bind(&BatteryPub::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Int32();
      //level -= 20;
      message.data = 12.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
      Battery_Pub->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr Battery_Pub;
    size_t count_;
    //int32_t level = 100;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryPub>());
  rclcpp::shutdown();
  return 0;
}