
#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class ElevatorUp : public plansys2::ActionExecutorClient
{
public:
  ElevatorUp()
  : plansys2::ActionExecutorClient("elevator_up", 2s)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 0.25) {
      progress_ += 0.25;
      send_feedback(progress_, "elevator_up running");
    } else {
      finish(false, 0.25, "elevator_up not_completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "elevator_up... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ElevatorUp>();

  node->set_parameter(rclcpp::Parameter("action_name", "elevator_up"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
