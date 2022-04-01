project(rescue_robot)

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
rclcpp
rclcpp_action
plansys2_msgs
plansys2_domain_expert
plansys2_executor
plansys2_planner
plansys2_problem_expert
plansys2_pddl_parser
ament_index_cpp
plansys2_bt_actions
)

################################################################################
#                         BATTERY ACTION NODES                                 #
################################################################################

add_executable(battery_charge_1_action_node src/battery/battery_charge_1_action_node.cpp)
ament_target_dependencies(battery_charge_1_action_node ${dependencies})

add_executable(charger_approach_action_node src/battery/charger_approach_action_node.cpp)
ament_target_dependencies(charger_approach_action_node ${dependencies})

################################################################################
#                    OBJECT HANDLING ACTION NODES                              #
################################################################################

add_executable(item_move_action_node src/object_handling/item_move_action_node.cpp)
ament_target_dependencies(item_move_action_node ${dependencies})

add_executable(item_victim_drop_action_node src/object_handling/item_victim_drop_action_node.cpp)
ament_target_dependencies(item_victim_drop_action_node ${dependencies})

add_executable(item_victim_grab_action_node src/object_handling/item_victim_grab_action_node.cpp)
ament_target_dependencies(item_victim_grab_action_node ${dependencies})

add_executable(victim_rescue_action_node src/object_handling/victim_rescue_action_node.cpp)
ament_target_dependencies(victim_rescue_action_node ${dependencies})

add_executable(obj_approach_action_node src/object_handling/obj_approach_action_node.cpp)
ament_target_dependencies(obj_approach_action_node ${dependencies})

add_executable(target_approach_action_node src/object_handling/target_approach_action_node.cpp)
ament_target_dependencies(target_approach_action_node ${dependencies})

################################################################################
#                         NAVIGATION ACTION NODES                              #
################################################################################

add_executable(elevator_approach_action_node src/navigation/elevator_approach_action_node.cpp)
ament_target_dependencies(elevator_approach_action_node ${dependencies})

add_executable(stairs_approach_action_node src/navigation/stairs_approach_action_node.cpp)
ament_target_dependencies(stairs_approach_action_node ${dependencies})

add_executable(elevator_up_action_node src/navigation/elevator_up_action_node.cpp)
ament_target_dependencies(elevator_up_action_node ${dependencies})

add_executable(elevator_down_action_node src/navigation/elevator_down_action_node.cpp)
ament_target_dependencies(elevator_down_action_node ${dependencies})

add_executable(stairs_up_action_node src/navigation/stairs_up_action_node.cpp)
ament_target_dependencies(stairs_up_action_node ${dependencies})

add_executable(stairs_down_action_node src/navigation/stairs_down_action_node.cpp)
ament_target_dependencies(stairs_down_action_node ${dependencies})

add_executable(door_open_action_node src/navigation/door_open_action_node.cpp)
ament_target_dependencies(door_open_action_node ${dependencies})

add_executable(door_destination_navigate_node src/navigation/door_destination_navigate_node.cpp)
ament_target_dependencies(door_destination_navigate_node ${dependencies})

add_executable(destination_navigate_node src/navigation/destination_navigate_node.cpp)
ament_target_dependencies(destination_navigate_node ${dependencies})

################################################################################
#                         BACKGROUND ACTION NODES                              #
################################################################################

add_executable(wander_around_action_node src/background/wander_around_action_node.cpp)
ament_target_dependencies(wander_around_action_node ${dependencies})

add_executable(obj_detect_start_action_node src/background/obj_detect_start_action_node.cpp)
ament_target_dependencies(obj_detect_start_action_node ${dependencies})

add_executable(explore_start_action_node src/background/explore_start_action_node.cpp)
ament_target_dependencies(explore_start_action_node ${dependencies})

add_executable(map_start_action_node src/background/map_start_action_node.cpp)
ament_target_dependencies(map_start_action_node ${dependencies})

################################################################################
#                         CONTROLLER ACTION NODES                              #
################################################################################

add_executable(controller_node src/controllers/controller_node.cpp)
ament_target_dependencies(controller_node ${dependencies})

add_executable(navigation_controller src/controllers/navigation_controller.cpp)
ament_target_dependencies(navigation_controller ${dependencies})

add_executable(object_handling_controller src/controllers/object_handling_controller.cpp)
ament_target_dependencies(object_handling_controller ${dependencies})



install(DIRECTORY launch pddl config behavior_trees_xml params DESTINATION share/${PROJECT_NAME})

install(TARGETS
  battery_charge_1_action_node
  charger_approach_action_node

  item_move_action_node
  item_victim_drop_action_node

  item_victim_grab_action_node
  obj_approach_action_node

  target_approach_action_node
  victim_rescue_action_node

  explore_start_action_node
  map_start_action_node

  wander_around_action_node
  elevator_approach_action_node

  stairs_approach_action_node
  elevator_down_action_node

  elevator_up_action_node
  elevator_approach_action_node

  stairs_up_action_node
  stairs_down_action_node

  door_open_action_node
  door_destination_navigate_node

  destination_navigate_node
  navigation_controller

  obj_detect_start_action_node
  object_handling_controller
 
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
