/*
    AUTHOR:         FRANKLINE OKONKWO
    LAST UPDATE:    31.03.2022
    EMAIL:          fokonkwo@mail.upb.de
 */

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
  : rclcpp::Node("object_handling_controller"), state(START)
  {
     battery_level_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/battery_pub", 1, std::bind(&ActionController::topic_callback, this, _1));
  }

  int32_t topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    //RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
    battery_level = msg->data;
    std::string battery_update = "= battery_level robot_1 " + std::to_string(battery_level);
          problem_expert_->updateFunction(plansys2::Function(battery_update));
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
      //Initialise Knowledge Base
      problem_expert_->addInstance(plansys2::Instance{"p1building", "world"});
      problem_expert_->addInstance(plansys2::Instance{"robot_1", "robot"});
      problem_expert_->addInstance(plansys2::Instance{"students", "victim"});

      problem_expert_->addPredicate(plansys2::Predicate("(item_victim_dropped robot_1 students)"));
      problem_expert_->addPredicate(plansys2::Predicate("(not_para robot_1)"));

      std::string f_battery_level = "= battery_level robot_1 " 
        + std::to_string(battery_level);
      std::string f_item_victim_detected = "= item_victim_detected students " 
        + std::to_string(item_victim_detected);
      std::string f_item_victim_at_target = "= item_victim_at_target students " 
        + std::to_string(item_victim_at_target);
      std::string f_item_victim_at_target_goal = "= item_victim_at_target_goal students " 
        + std::to_string(item_victim_at_target_goal);

      problem_expert_->addFunction(plansys2::Function(f_battery_level));
      problem_expert_->addFunction(plansys2::Function(f_item_victim_detected));
      problem_expert_->addFunction(plansys2::Function(f_item_victim_at_target));
      problem_expert_->addFunction(plansys2::Function(f_item_victim_at_target_goal));

      problem_expert_->setGoal(plansys2::Goal("(and(victim_rescued robot_1 students))"));


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
          break;
        }
        else
        {
          std::cout << "--------------------------------------------------------------------" << std::endl;
          std::cout << "Plan found to reach goal:" << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
          std::cout << "--------------------------------------------------------------------" << std::endl;
          executor_client_->start_plan_execution(plan.value());
          state = RUN;
          break;
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
          

        //subscribe to battery_level topic to update battery funtion
        
        
        //if(battery_level < 12.0 ){
         
          //std::string function_update = "= battery_level robot_1 " + std::to_string(battery_level);
         // problem_expert_->updateFunction(plansys2::Function(function_update));
         // problem_expert_->clearGoal();
       // }
        
        std::vector<plansys2::Function> functions = problem_expert_->getFunctions();
        for (const auto &function : functions)
          {
            std::cout<< function.name<<":"<< function.value<<std::endl;
            
            if (function.name == "battery_level")
            {
              double battery_level = function.value;
              if (battery_level < 15.0 && !re_planning)
              {
                RCLCPP_WARN(get_logger(), "BATTERY LOW");
                RCLCPP_WARN(get_logger(), "CANCELLING PLAN AND REPLAN");

                RCLCPP_WARN(get_logger(), "SETTING GOAL CHARGE BATTERY");
                //problem_expert_->clearGoal();
                executor_client_->cancel_plan_execution();
                //problem_expert_->clearKnowledge();
                state = Updateknowledge;
                break;
              }
            }
          }

        // Plan finished
        if (!executor_client_->execute_and_check_plan())
        {  
          auto result = executor_client_->getResult();

          if (result.value().success)
          {

            RCLCPP_INFO(get_logger(), "Plan succesfully finished");

          } else {
            RCLCPP_ERROR(get_logger(), "Plan finished with error");
          }
        }
      }
      break;

      case Updateknowledge:
      {
        //updating Funtion Values

        std::cout << "--------------------------------------------------------------------" << std::endl;
        std::cout << "REPLANNING" << std::endl;
        std::cout << "--------------------------------------------------------------------" << std::endl;

        std::vector<plansys2::Function> functions = problem_expert_->getFunctions();
        for (const auto &function : functions)
        {
         
          if(function.name == "battery_level")
          {
            std::string function_update = "= battery_level robot_1 " 
              + std::to_string(function.value);
            problem_expert_->updateFunction(plansys2::Function(function_update));
          }
          
          if(function.name == "item_victim_detected")
          {
            
            std::string f_item_victim_detected = "= item_victim_detected students " 
                + std::to_string(function.value);
            problem_expert_->updateFunction(plansys2::Function(f_item_victim_detected));
        
          }

          if(function.name == "item_victim_at_target")
          {
            std::string f_item_victim_at_target = "= item_victim_at_target students " 
                + std::to_string(function.value);
            problem_expert_->updateFunction(plansys2::Function(f_item_victim_at_target));
          
          }

          if(function.name == "item_victim_at_target_goal")
          {
            std::string f_item_victim_at_target_goal = "= item_victim_at_target_goal students " 
                + std::to_string(function.value);
            problem_expert_->updateFunction(plansys2::Function(f_item_victim_at_target_goal));
           
          }
        }
        re_planning = true;
        state = START;
        break;
      }
      break;

      case RESCUE:
      {
        //problem_expert_->addInstance(plansys2::Instance{"robot_1", "robot"});
        
        problem_expert_->setGoal(plansys2::Goal("(and(victim_rescued robot_1 students))"));
        state = START;
      }
      break;

      case SOS:
      {
        //problem_expert_->addInstance(plansys2::Instance{"robot_1", "robot"});
        
        problem_expert_->setGoal(plansys2::Goal("(and(victim_rescued robot_1 students))"));
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
    {START, CHARGE, RUN, RESCUE, Updateknowledge, SOS } StateType;
    StateType state;

  bool check = false;
  bool re_planning = false;
  bool re_plan = false;

  int32_t battery_level = 100;
  int32_t item_victim_detected = 5;
  int32_t item_victim_at_target_goal = 5;
  int32_t item_victim_at_target = 0.0;
  
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
