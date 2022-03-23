set instance P6Building world

set instance robot_1 robot
set instance robot_2 robot
set instance robot_3 robot

set instance redBalls balls
set instance blueBalls balls
set instance whiteBoxes boxes

set predicate (items_dropped robot_1 redBalls)
set predicate (items_dropped robot_2 blueBalls)
set predicate (items_dropped robot_3 whiteBoxes)

set function (= (battery_level robot_1) 100)
set function (= (battery_level robot_2) 100)
set function (= (battery_level robot_3) 100)

set function (= (detected_items redBalls) 10)
set function (= (detected_items blueBalls) 10)
set function (= (detected_items whiteBoxes) 10)

set function (= (at_target_items redBalls) 0)
set function (= (at_target_items blueBalls) 0)
set function (= (at_target_items whiteBoxes) 0)

set function (= (carried_items robot_1) 0)
set function (= (carried_items robot_2) 0)
set function (= (carried_items robot_3) 0)

set function (= (at_target_items_goal redBalls) 2)
set function (= (at_target_items_goal blueBalls) 1)
set function (= (at_target_items_goal whiteBoxes) 0)

set goal (and (items_handeled robot_1 redBalls) (items_handeled robot_2 blueBalls) (items_handeled robot_3 whiteBoxes))
