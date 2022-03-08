#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
using std::placeholders::_1;

class BatterySub : public rclcpp::Node
{
  public:
    BatterySub()
    : Node("battery_sub")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "battery_pub", 10, std::bind(&BatterySub::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatterySub>());
  rclcpp::shutdown();
  return 0;
}