
#include <memory>
#include <random>
#include <list>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <chrono>

#include <plansys2_pddl_parser/Utils.h>
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;



class Controller : public rclcpp::Node
{
public:
  Controller()
      : rclcpp::Node("behavoir_controller"), state(STARTING)
  {
    battery_level_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/battery_pub", 1, std::bind(&Controller::topic_callback, this, _1));

  }

  int32_t topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Battery Level: '%d'", msg->data);
    battery_level = msg->data;
    return battery_level;
  }

  

  void init()
  {

    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    init_knowledge();
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"w1", "world"});
    problem_expert_->addInstance(plansys2::Instance{"r1", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"b1", "balls"});

    problem_expert_->addPredicate(plansys2::Predicate("(balls_dropped r1 b1)"));

    problem_expert_->addFunction(plansys2::Function("= battery_level r1 1000"));
    problem_expert_->addFunction(plansys2::Function("= detected_balls b1 5"));
    problem_expert_->addFunction(plansys2::Function("= at_target_balls b1 0"));
    problem_expert_->addFunction(plansys2::Function("= carried_balls r1 0"));
    problem_expert_->addFunction(plansys2::Function("= at_target_balls_goal b1 2"));
  
  }
  void step()
  {
    switch (state)
    {
    case STARTING:
    {
      problem_expert_->setGoal(plansys2::Goal("(and(balls_handeled r1 b1))"));

      // compute plan
      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      if (plan.has_value())
      {
        std::cout << "Plan Found to Reach Goal:" << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;

        executor_client_->execute_and_check_plan();
        std::cout << "EXECUTING PLAN " << std::endl;
        executor_client_->start_plan_execution(plan.value());
        state = RUN;
      }
    }
    break;

    case RUN:
    {
      auto feedback = executor_client_->getFeedBack();
      for (const auto &action_feedback : feedback.action_execution_status)
      {

        RCLCPP_INFO_STREAM(get_logger(), "[" << action_feedback.action << " " << action_feedback.completion * 100.0 << "%]");
      }
      std::cout << std::endl;

      std::vector<plansys2::Function> functions = problem_expert_->getFunctions();

      for (const auto &function : functions)
      {
        if (function.name == "battery_level")
        {
          double battery_level = function.value;
          if (battery_level < 90)
          {

            RCLCPP_WARN(get_logger(), "BATTERY LOW");
            RCLCPP_WARN(get_logger(), "CANCELLING PLAN AND REPLAN");
            RCLCPP_WARN(get_logger(), "SETTING GOAL CHARGE BATTERY");

            executor_client_->cancel_plan_execution();
            problem_expert_->clearKnowledge();
            problem_expert_->clearGoal();
            state = REPLAN_2;
          }
        }
      }

      if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
      {
        if (executor_client_->getResult().value().success)
        {
          std::cout << "GOAL SUCESSFULL" << std::endl;
          std::cout << "SETTING NEW GOAL" << std::endl;
          problem_expert_->clearKnowledge();
          problem_expert_->clearGoal();
          state = REPLAN_2;
        }
      }
    }
    break;

    case REPLAN_2:
    {
      problem_expert_->addInstance(plansys2::Instance{"getbot", "robot"});
      problem_expert_->addInstance(plansys2::Instance{"entrance", "room"});
      problem_expert_->addInstance(plansys2::Instance{"first_floor", "room"});
      problem_expert_->addInstance(plansys2::Instance{"charging_room", "room"});

      problem_expert_->addPredicate(plansys2::Predicate("(connected entrance first_floor)"));
      problem_expert_->addPredicate(plansys2::Predicate("(connected first_floor entrance)"));

      problem_expert_->addPredicate(plansys2::Predicate("(connected charging_room first_floor)"));
      problem_expert_->addPredicate(plansys2::Predicate("(connected first_floor charging_room)"));
      problem_expert_->addPredicate(plansys2::Predicate("(battery_low getbot)"));

      problem_expert_->addPredicate(plansys2::Predicate("(battery_low getbot)"));
      problem_expert_->addPredicate(plansys2::Predicate("(robot_at getbot entrance)"));
      problem_expert_->addPredicate(plansys2::Predicate("(charging_point_at charging_room)"));

      problem_expert_->setGoal(plansys2::Goal("(and(robot_at getbot first_floor))"));

      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      
      if (plan.has_value())
      {
        std::cout << "Plan Found to Reach Goal: " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;

        executor_client_->execute_and_check_plan();
        std::cout << "EXECUTING PLAN " << std::endl;
        executor_client_->start_plan_execution(plan.value());
        state = REPLAN_3;
      }
      else
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Could not find plan to reach goal " << 
          parser::pddl::toString(problem_expert_->getGoal()));

          RCLCPP_WARN(get_logger(), "REPLANNING");
          RCLCPP_WARN(get_logger(), "UPDATING STATE");
        
        state = REPLAN_4;
        break;
      }
    }
    break;

    case REPLAN_3:

    {
      if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
      {
        if (executor_client_->getResult().value().success)
        {
          std::cout << "GOAL SUCESSFULL" << std::endl;
        }
      }
    }
    break;

    case REPLAN_4:
    {
      problem_expert_->clearKnowledge();
      problem_expert_->clearGoal();

      problem_expert_->addInstance(plansys2::Instance{"getbot", "robot"});
      problem_expert_->addInstance(plansys2::Instance{"entrance", "room"});
      problem_expert_->addInstance(plansys2::Instance{"first_floor", "room"});
      problem_expert_->addInstance(plansys2::Instance{"charging_room", "room"});

      problem_expert_->addPredicate(plansys2::Predicate("(connected entrance first_floor)"));
      problem_expert_->addPredicate(plansys2::Predicate("(connected first_floor entrance)"));

      problem_expert_->addPredicate(plansys2::Predicate("(connected charging_room first_floor)"));
      problem_expert_->addPredicate(plansys2::Predicate("(connected first_floor charging_room)"));
      problem_expert_->addPredicate(plansys2::Predicate("(battery_low getbot)"));

      problem_expert_->addPredicate(plansys2::Predicate("(battery_low getbot)"));
      problem_expert_->addPredicate(plansys2::Predicate("(robot_at getbot entrance)"));
      problem_expert_->addPredicate(plansys2::Predicate("(charging_point_at charging_room)"));

      problem_expert_->setGoal(plansys2::Goal("(and(battery_full getbot))"));

      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      
      if (plan.has_value())
      {
        std::cout << "Plan Found to Reach Goal: " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;

        executor_client_->execute_and_check_plan();
        std::cout << "EXECUTING PLAN " << std::endl;
        executor_client_->start_plan_execution(plan.value());
        state = REPLAN_3;
      }
    }
    break;

    case REPLAN_5:

    {
      if (!executor_client_->execute_and_check_plan() && executor_client_->getResult())
      {
        if (executor_client_->getResult().value().success)
        {
          std::cout << "GOAL SUCESSFULL" << std::endl;
        }
      }
    }
    break;
    default:
      break;
    }
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  rclcpp::TimerBase::SharedPtr waiting_timer_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr battery_level_sub_;

    typedef enum
    {
      STARTING,
      RUN,
      REPLAN_1,
      REPLAN_2,
      REPLAN_3,
      REPLAN_4,
      REPLAN_5
    } StateType;
    StateType state;

    bool check = false;
    int32_t battery_level = 0;
};

int main(int argc, char **argv)
{
 rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();

  node->init();

  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
    node->step();
  }

  rclcpp::shutdown();

  return 0;
}
