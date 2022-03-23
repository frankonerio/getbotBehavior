Background Domain Problems


1. Exploring a world with a pre-loaded map and full battery

set instance p1building world
set instance robot_1 robot
set predicate (map_available robot_1 p1building)
set function (= (battery_level robot_1) 100)
set goal (and (exploring robot_1 p1building))
get plan

2. Continue mapping a world with a pre-loaded map and full battery

set instance P2Building world
set instance robot_2 robot
set predicate (map_available robot_2 P2Building)
set function (= (battery_level robot_2) 100)
set goal (and (mapping robot_2 P2Building))
get plan

3. Start Mapping in a completely new world and full battery (Problem)

set instance P3Building world
set instance robot_3 robot
set function (= (battery_level robot_3) 10)
set goal (and (mapping robot_3 P3Building))
get plan




4. Start Mapping in a completely new world detect objectes in it with very low battery

set instance P4Building world
set instance robot_4 robot
set function (= (battery_level robot_4) 10)
set goal (and (mapping robot_4 P4Building) (obj_detecting robot_4 P4Building))
get plan


problem_expert_->addInstance(plansys2::Instance{"p4Building", "world"});
      problem_expert_->addInstance(plansys2::Instance{"robot_4", "robot"});
 

    //problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used steering_wheel_3)"));

      problem_expert_->addFunction(plansys2::Function("= battery_level robot_4 10"));

      problem_expert_->setGoal(plansys2::Goal("(and(mapping robot_4 P4Building) (obj_detecting robot_4 P4Building)"));