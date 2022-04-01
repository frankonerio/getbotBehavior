#### object handling Problem description###

set instance P1Building world
set instance robot_1 robot
set instance students victim

set predicate (item_victim_dropped robot_1 students)
set predicate (not_para robot_1)

set function (= (battery_level robot_1) 100)
set function (= (item_victim_detected students) 2)
set function (= (item_victim_at_target_goal students) 2)

set goal (and (victim_rescued robot_1 students))
get plan

### laucnh controller ###
. install/setup.bash
ros2 run rescue_robot object_handling_controller

### trigger low battery ###
call battery publisher node.
. install/setup.bash
ros2 run battery_pub battery_pub
publishes battery level = 11 on topic /battery_pub to
trigger replanning