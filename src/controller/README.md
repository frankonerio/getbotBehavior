set instance p6Building world
set instance robot_1 robot
set instance blue_balls balls

set predicate (balls_dropped robot_1 blue_balls)

set function (= (battery_level robot_1) 100)
set function (= (detected_balls blue_balls) 10)
set function (= (at_target_balls blue_balls 0)
set function (= (carried_balls robot_1) 0)

set function (= (at_target_balls_goal blue_balls) 7)
set goal (and(balls_handeled robot_1 blue_balls))

get plan



set instance P6Building world
set instance robot_1 robot
set instance HDD_Boxes box

set predicate (item_victim_dropped robot_1 HDD_Boxes)

set function (= (battery_level robot_1) 100)
set function (= (item_victim_detected HDD_Boxes) 5)
set function (= (item_victim_at_target HDD_Boxes) 0)
set function (= (item_victim_carried robot_1) 0)
set function (= (item_victim_at_target_goal HDD_Boxes) 5)

set goal (and (item_moved robot_1 HDD_Boxes))
