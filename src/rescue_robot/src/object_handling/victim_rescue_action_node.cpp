
#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class VictimRescue : public plansys2::ActionExecutorClient
{
public:
  VictimRescue()
  : plansys2::ActionExecutorClient("victim_rescue", 2s)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 2.0) {
      progress_ += 0.5;
      send_feedback(progress_, "victim_rescue running");
    } else {
      finish(true, 1.0, "victim_rescue completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "victim_rescue ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VictimRescue>();

  node->set_parameter(rclcpp::Parameter("action_name", "victim_rescue"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
