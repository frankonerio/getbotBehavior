#include <memory>
#include <random>
#include <list>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include <plansys2_pddl_parser/Utils.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class NavigationController : public rclcpp::Node
{
public:
NavigationController()
: rclcpp::Node("navigation_controller"), state(GOAL_4)
{
}

void init()
{
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

}
//initialising state in knowldege base
void init_knowledge()
{
    problem_expert_->addInstance(plansys2::Instance{"p1building", "world"});
    problem_expert_->addInstance(plansys2::Instance{"robot_1", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"main_stairs", "stairs"});
    problem_expert_->addInstance(plansys2::Instance{"main_elevator", "elevator"});

}

void computePlan()
{
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);
}

void step()
{
switch (state) {
    case GOAL_1:
    {

        init_knowledge();

        problem_expert_->addInstance(plansys2::Instance{"control_room_1", "destination"});

        problem_expert_->addPredicate(plansys2::Predicate("(stairs_usable main_stairs p1building)"));
        problem_expert_->addPredicate(plansys2::Predicate("(no_door_inway control_room_1)"));

        problem_expert_->addFunction(plansys2::Function("= current_floor robot_1 p1building 2"));
        problem_expert_->addFunction(plansys2::Function("= destination_floor control_room_1 p1building 6"));

        // Set the goal for next state
        problem_expert_->setGoal(plansys2::Goal("(and(destination_reached robot_1 control_room_1))"));
            
        // Compute the plan
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
        }else{

            std::cout << "--------------------------------------------------------------------" << std::endl;
            std::cout << "Plan found to reach goal:" << 
            parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            std::cout << "--------------------------------------------------------------------" << std::endl;
        }

          // Execute the plan
        if (executor_client_->start_plan_execution(plan.value())) {
            state = RUN;
            currentstate = GOAL_1;
        }
    }  
    break;
    
    case GOAL_2:
    {
        init_knowledge();
        problem_expert_->addInstance(plansys2::Instance{"control_room_2", "destination"});
        problem_expert_->addPredicate(plansys2::Predicate("(elevator_usable main_elevator p1building)"));
        problem_expert_->addPredicate(plansys2::Predicate("(stairs_usable main_stairs p1building)"));
        problem_expert_->addPredicate(plansys2::Predicate("(no_door_inway control_room_2)"));

        
        problem_expert_->addFunction(plansys2::Function("= current_floor robot_1 p1building 6"));
        problem_expert_->addFunction(plansys2::Function("= destination_floor control_room_2 p1building 3"));
        

        // Set the goal for next state
        problem_expert_->setGoal(plansys2::Goal("(and(destination_reached robot_1 control_room_2))"));
            
        // Compute the plan
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
        }else{
            std::cout << "--------------------------------------------------------------------" << std::endl;
            std::cout << "Plan found to reach goal:" << 
            parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            std::cout << "--------------------------------------------------------------------" << std::endl;
        }

          // Execute the plan
        if (executor_client_->start_plan_execution(plan.value())) {
            laststate = GOAL_1;
            currentstate = GOAL_2;
            state = RUN;
        }    
    }  
    break;

    case GOAL_3:
    {
        init_knowledge();
        problem_expert_->addInstance(plansys2::Instance{"control_room_3", "destination"});
        problem_expert_->addInstance(plansys2::Instance{"control_room_3_door", "door"});

        problem_expert_->addPredicate(plansys2::Predicate("(elevator_usable main_elevator p1building)"));
        problem_expert_->addPredicate(plansys2::Predicate("(stairs_usable main_stairs p1building)"));
        problem_expert_->addPredicate(plansys2::Predicate("(door_inway control_room_3_door control_room_3)"));
        problem_expert_->addPredicate(plansys2::Predicate("(door_opened control_room_3_door control_room_3)"));
        
        problem_expert_->addFunction(plansys2::Function("= current_floor robot_1 p1building 3"));
        problem_expert_->addFunction(plansys2::Function("= destination_floor control_room_3 p1building 0"));

        problem_expert_->setGoal(plansys2::Goal("(and(destination_reached robot_1 control_room_3))"));   

        // Compute the plan
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
        }else{

            std::cout << "--------------------------------------------------------------------" << std::endl;
            std::cout << "Plan found to reach goal:" << 
            parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            std::cout << "--------------------------------------------------------------------" << std::endl;
        }

          // Execute the plan
        if (executor_client_->start_plan_execution(plan.value())) {
            laststate = GOAL_2;
            currentstate = GOAL_3;
            state = RUN;
        }
    }
    break;

    case GOAL_4:
    {
        init_knowledge();
        problem_expert_->addInstance(plansys2::Instance{"control_room_4", "destination"});
        problem_expert_->addInstance(plansys2::Instance{"control_room_4_door", "door"});

        problem_expert_->addPredicate(plansys2::Predicate("(stairs_usable main_stairs p1building)"));
        problem_expert_->addPredicate(plansys2::Predicate("(elevator_usable main_elevator p1building)"));
        problem_expert_->addPredicate(plansys2::Predicate("(door_inway control_room_4_door control_room_4)"));
        
        
        problem_expert_->addFunction(plansys2::Function("= current_floor robot_1 p1building 0"));
        problem_expert_->addFunction(plansys2::Function("= destination_floor control_room_4 p1building 2"));

        problem_expert_->setGoal(plansys2::Goal("(and(destination_reached robot_1 control_room_4))"));  

         // Compute the plan
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << "GENERATING PLAN" << std::endl;
        std::cout << "-------------------------------------------------------------------------" << std::endl;


        if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
        }else{

            std::cout << "--------------------------------------------------------------------------" << std::endl;
            std::cout << "Plan found to reach goal:" << 
            parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            std::cout << "----------------------------------------------------------------------------" << std::endl;
        }

        if (executor_client_->start_plan_execution(plan.value())) {
            laststate = GOAL_3;
            currentstate = GOAL_4;
            state = RUN;
        }
 
    }
    break;

    case RUN:
    {
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << "EXECUTING GENERATED PLAN" << std::endl;
        std::cout << "-------------------------------------------------------------------------" << std::endl;


        auto feedback = executor_client_->getFeedBack();
        for (const auto &action_feedback : feedback.action_execution_status){

            RCLCPP_INFO_STREAM(get_logger(), "[" << action_feedback.action 
                << " " << action_feedback.completion * 100.0 << "%]");
            if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;

                //std::cout << "--------------------------------------------------------------------" << std::endl;
                //std::cout << "REMOVING ELEVATOR INSTANCES AND PREDICATES" << std::endl;
                //std::cout << "--------------------------------------------------------------------" << std::endl;

                problem_expert_->removePredicate(plansys2::Predicate("(elevator_usable main_elevator p1building)"));
                problem_expert_->removeInstance(plansys2::Instance{"main_elevator", "elevator"});
                
                //cancel plan execution
                
                //problem_expert_->clearGoal();
                executor_client_->cancel_plan_execution();

                state = REPLAN;
                break;
            }
        } 
        std::cout << std::endl;

        if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
                std::cout << "Successful finished " << std::endl;
                problem_expert_->clearGoal();
                std::cout << "SETTING NEW GOAL" << std::endl;
                std::cout << "--------------------------------------------------------------------" << std::endl;
                std::cout << "UPDATING KNOWLEDGE BASE" << std::endl;
                std::cout << "--------------------------------------------------------------------" << std::endl;
                problem_expert_->clearKnowledge();
                //problem_expert_->removeFunction(plansys2::Function("= destination_floor control_room_1 p1building 6"));
                //problem_expert_->removeFunction(plansys2::Function("= current_floor robot_1 p1building 2"));
                //problem_expert_->removePredicate(plansys2::Predicate("(stairs_usable main_stairs p1building)"));
                //problem_expert_->removePredicate(plansys2::Predicate("(no_door_inway control_room_1)"));
                //problem_expert_->removeInstance(plansys2::Instance{"control_room_1", "destination"});
                
            }

            if(currentstate == GOAL_1 && state == RUN){
                state = GOAL_2;
                break;
            }
            if(currentstate == GOAL_2 && laststate == GOAL_1){
                state = GOAL_3;
            }
            if(currentstate == GOAL_3 && laststate == GOAL_2){
                state = GOAL_4;
            }
            if(currentstate == GOAL_4 && laststate == GOAL_3 && state != REPLAN){
                state = IDLE;
            }
            break;
        }
    }   
    break;

    case REPLAN:
    {
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << "REMOVING ELEVATOR INSTANCES AND PREDICATES" << std::endl;
        std::cout << "-------------------------------------------------------------------------" << std::endl;

        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << "REPLANNING" << std::endl;
        std::cout << "-------------------------------------------------------------------------" << std::endl;
        

        if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
        }else{

            std::cout << "--------------------------------------------------------------------------" << std::endl;
            std::cout << "Plan found to reach goal:" << 
            parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            std::cout << "----------------------------------------------------------------------------" << std::endl;
        }

        if (executor_client_->start_plan_execution(plan.value())) {
        state = RUN;
        }
        break;
    }
    break;

    case IDLE:
    {

    }
    break;
    default:
    break;
    }
}

private:
    typedef enum {START, RUN, GOAL_1, GOAL_2, GOAL_3, GOAL_4, GOAL_5, REPLAN, IDLE} goal;
    goal state;
    goal laststate;
    goal currentstate;
    goal nextstate;

    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationController>();

    node->init();

    rclcpp::Rate rate(2);
    while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
    }

rclcpp::shutdown();

return 0;
}