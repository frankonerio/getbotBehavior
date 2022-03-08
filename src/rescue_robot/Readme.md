set instance P6Building world

set instance robot_1 robot
set instance robot_2 robot

set instance HDD_Boxes ball
set instance Students victim

set predicate (item_victim_dropped robot_1 HDD_Boxes)
set predicate (item_victim_dropped robot_2 Students)

set function (= (battery_level robot_1) 100)
set function (= (battery_level robot_2) 100)

set function (= (item_victim_detected HDD_Boxes) 10)
set function (= (item_victim_detected Students) 10)

set function (= (item_victim_at_target HDD_Boxes) 0)
set function (= (item_victim_at_target Students) 0)

set function (= (item_victim_carried robot_1) 0)
set function (= (item_victim_carried robot_2) 0)

set function (= (item_victim_at_target_goal HDD_Boxes) 5)
set function (= (item_victim_at_target_goal Students) 2)

set goal (and (item_moved robot_1 HDD_Boxes) (victim_rescued robot_2 Students))
get plan


###################### single robot ######################################

set instance P6Building world

set instance robot_1 robot
set instance students victim

set predicate (item_victim_dropped robot_1 students)

set function (= (battery_level robot_1) 1000)
set function (= (item_victim_detected students) 10)
set function (= (item_victim_at_target students) 0)
set function (= (item_victim_carried robot_1) 0)
set function (= (item_victim_at_target_goal students) 5)

set goal (and (victim_rescued robot_1 students))
get plan

#######################################################
set instance P6Building world
set instance robot_1 robot
set instance HDD_Boxes box

set predicate (item_victim_dropped robot_1 HDD_Boxes)

set function (= (battery_level robot_1) 100)
set function (= (item_victim_detected HDD_Boxes) 2)
set function (= (item_victim_at_target HDD_Boxes) 0)
set function (= (item_victim_carried robot_1) 0)
set function (= (item_victim_at_target_goal HDD_Boxes) 2)

set goal (and (item_moved robot_1 HDD_Boxes))
