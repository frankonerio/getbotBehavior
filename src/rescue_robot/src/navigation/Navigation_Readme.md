navigation domain problems

problem introduction for a robot navigating between different rooms in different floors, 
under different conditions:

set instance p1building world
set instance robot_1 robot
set instance main_stairs stairs
set instance main_elevator elevator



1. problem 1:

robot at floor 0 to navigate up to floor 6 to reach “control room 1” that doesn’t have a door.
only stairs are usable at the moment.

set instance control_room_1 destination
set predicate (stairs_usable main_stairs p1building)
set predicate (no_door_inway control_room_1)
set function (= (current_floor robot_1 p1building) 2)
set function (= (destination_floor control_room_1 p1building) 6)

set goal (and (destination_reached robot_1 control_room_1))
get plan


2. problem 2:

continuing same problem,
robot at floor 6 to navigate down to floor 3 to reach “control room 2” that doesn’t have a door.
now both elevator and stairs are usable at the moment.

set instance control_room_2 destination
set predicate (elevator_usable main_elevator p1building)
set predicate (stairs_usable main_stairs p1building)
set predicate (no_door_inway control_room_2)
set function (= (current_floor robot_1 p1building) 6)
set function (= (destination_floor control_room_2 p1building) 3)

set goal (and (destination_reached robot_1 control_room_2))
get plan




3. problem 3:

continuing same problem,
robot at floor 3 to navigate down to floor 0 to reach “control room 3” that has a door.
however, the door is open. now both elevator and stairs are usable at the moment.

set instance control_room_3 destination
set instance control_room_3_door door
set predicate (elevator_usable main_elevator p1building)
set predicate (stairs_usable main_stairs p1building)
set predicate (elevator_usable main_elevator p1building)
set predicate (door_inway control_room_3_door control_room_3)
set predicate (door_opened control_room_3_door control_room_3)
set function (= (current_floor robot_1 p1building) 3)
set function (= (destination_floor control_room_3 p1building) 0)

set goal (and (destination_reached robot_1 control_room_3))
get plan


4. problem 4:

continuing same problem,
robot at floor 0 to navigate up to floor 0 to reach “control room 3” that has a door.
however, the door is open. now both elevator and stairs are usable at the moment.

set instance control_room_4 destination
set instance control_room_4_door door
set predicate (elevator_usable main_elevator p1building)
set predicate (stairs_usable main_stairs p1building)
set predicate (elevator_usable main_elevator p1building)
set predicate (door_inway control_room_4_door control_room_4)
set function (= (current_floor robot_1 p1building) 0)
set function (= (destination_floor control_room_4 p1building) 2)

set goal (and (destination_reached robot_1 control_room_4))
get plan


