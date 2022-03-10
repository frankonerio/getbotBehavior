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
class ActionController : public rclcpp::Node
{
public:
  ActionController()
  : rclcpp::Node("action_controller"), state(RESCUE)
  {
     battery_level_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/battery_pub", 1, std::bind(&ActionController::topic_callback, this, _1));

  }

  int32_t topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    //RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
    battery_level = msg->data;
    return battery_level;
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    init_knowledge();
    return true;
  }

  void init_knowledge()
  {
      //
      problem_expert_->addInstance(plansys2::Instance{"r1", "robot"});

      problem_expert_->addInstance(plansys2::Instance{"entrance", "room"});
      problem_expert_->addInstance(plansys2::Instance{"first_floor", "room"});
      problem_expert_->addInstance(plansys2::Instance{"charging_room", "room"});

      problem_expert_->addPredicate(plansys2::Predicate("(connected entrance first_floor)"));
      problem_expert_->addPredicate(plansys2::Predicate("(connected first_floor entrance)"));

      problem_expert_->addPredicate(plansys2::Predicate("(connected charging_room first_floor)"));
      problem_expert_->addPredicate(plansys2::Predicate("(connected first_floor charging_room)"));
      //problem_expert_->addPredicate(plansys2::Predicate("(battery_low r1)"));

      
      problem_expert_->addPredicate(plansys2::Predicate("(robot_at r1 entrance)"));
      problem_expert_->addPredicate(plansys2::Predicate("(charging_point_at charging_room)"));

      //
      problem_expert_->addInstance(plansys2::Instance{"w1", "world"});
      //problem_expert_->addInstance(plansys2::Instance{"r1", "robot"});
      problem_expert_->addInstance(plansys2::Instance{"b1", "balls"});

      problem_expert_->addPredicate(plansys2::Predicate("(balls_dropped r1 b1)"));

      std::string f_battery_level = "= battery_level r1 " + std::to_string(battery_level);
      std::string f_detected_balls = "= detected_balls b1 " + std::to_string(detected_balls);
      std::string f_at_target_balls = "= at_target_balls b1 " + std::to_string(at_target_balls);
      std::string f_carried_balls = "= carried_balls r1 " + std::to_string(carried_balls);
      std::string f_at_target_balls_goal = "= at_target_balls_goal b1 " + std::to_string(at_target_balls_goal);

      problem_expert_->addFunction(plansys2::Function(f_battery_level));
      problem_expert_->addFunction(plansys2::Function(f_detected_balls));
      problem_expert_->addFunction(plansys2::Function(f_at_target_balls));
      problem_expert_->addFunction(plansys2::Function(f_carried_balls));
      problem_expert_->addFunction(plansys2::Function(f_at_target_balls_goal));

      //problem_expert_->setGoal(plansys2::Goal("(and(balls_handeled r1 b1))"));
  }

  void step()
  {
    switch (state)
    {
      case START:
      {
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        if (!plan.has_value()) {
          std::cout << "Could not find plan to reach goal " <<
            parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        }else{
          std::cout << "Plan found to reach goal:" << 
            parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
          state = RUN;
        }

        if (!executor_client_->start_plan_execution(plan.value())) {
          RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
        }
        break;
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

        //subscribe to battery_level topic to update battery funtion
        
        if(battery_level < 10){
         
          std::string function_update = "= battery_level r1 " + std::to_string(battery_level);
          problem_expert_->updateFunction(plansys2::Function(function_update));
          //problem_expert_->clearGoal();
        }

        std::vector<plansys2::Function> functions = problem_expert_->getFunctions();
        for (const auto &function : functions)
          {
            std::cout<< function.name<<":"<< function.value<<std::endl;
            
            if (function.name == "battery_level")
            {
              double battery_level = function.value;
              if (battery_level < 10.0 && !charging)
              {
                RCLCPP_WARN(get_logger(), "BATTERY LOW");
                RCLCPP_WARN(get_logger(), "CANCELLING PLAN AND REPLAN");

                RCLCPP_WARN(get_logger(), "SETTING GOAL CHARGE BATTERY");
                problem_expert_->clearGoal();
                executor_client_->cancel_plan_execution();
                //problem_expert_->clearKnowledge();
                state = CHARGE;
              }
            }
          }

        if (!executor_client_->execute_and_check_plan()) {  // Plan finished
          auto result = executor_client_->getResult();

          if (result.value().success) {

            RCLCPP_INFO(get_logger(), "Plan succesfully finished");
            //if(replan){
              problem_expert_->clearGoal();
              battery_level = 100;

               std::vector<plansys2::Function> functions = problem_expert_->getFunctions();
              for (const auto &function : functions)
              {
                std::string function_update = "= battery_level r1 " + std::to_string(battery_level);
                problem_expert_->updateFunction(plansys2::Function(function_update));

            //remove block

                if(function.name == "detected_balls"){
                  std::string f_detected_balls = "= detected_balls b1 " + std::to_string(function.value);
                  problem_expert_->updateFunction(plansys2::Function(f_detected_balls));
                }

                if(function.name == "at_target_balls"){
                  std::string f_at_target_balls = "= at_target_balls b1 " + std::to_string(function.value);
                  problem_expert_->updateFunction(plansys2::Function(f_at_target_balls));
                }

                if(function.name == "carried_ball"){
                  std::string f_carried_balls = "= carried_balls r1 " + std::to_string(function.value);
                  problem_expert_->updateFunction(plansys2::Function(f_carried_balls));
                }
              
                if(function.name == "at_target_balls_goal"){
                  std::string f_at_target_balls_goal = "= at_target_balls_goal b1 " + std::to_string(function.value);
                  problem_expert_->updateFunction(plansys2::Function(f_at_target_balls_goal));
                }
              }
            //removeblock
              state = RESCUE;
              break;
            //}
          
            //break;

          } else {
            RCLCPP_ERROR(get_logger(), "Plan finished with error");
          }
        }
      }
      break;

      case CHARGE:
      {
        //problem_expert_->addInstance(plansys2::Instance{"r1", "robot"});
        problem_expert_->addPredicate(plansys2::Predicate("(battery_low r1)"));
        problem_expert_->setGoal(plansys2::Goal("(and(battery_full r1))"));

        replan = true;
        charging = true;
        state = START;
      }
      break;

      case RESCUE:
      {
        //problem_expert_->addInstance(plansys2::Instance{"r1", "robot"});
        
        problem_expert_->setGoal(plansys2::Goal("(and(balls_handeled r1 b1))"));

        charging = false;
        state = START;
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

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr battery_level_sub_;

  typedef enum
    {START, CHARGE, RUN, RESCUE} StateType;
    StateType state;

  bool check = false;
  bool charging = true;
  bool replan = false;

  int32_t battery_level = 100;
  int32_t detected_balls = 5;
  int32_t at_target_balls_goal = 2;
  int32_t at_target_balls = 0;
  int32_t carried_balls = 0;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionController>();

  if (!node->init()) {
    return 0;
  }

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
