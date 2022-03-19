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

add_executable(battery_charge_1_action_node src/battery_charge_1_action_node.cpp)
ament_target_dependencies(battery_charge_1_action_node ${dependencies})

add_executable(charger_approach_action_node src/charger_approach_action_node.cpp)
ament_target_dependencies(charger_approach_action_node ${dependencies})

add_executable(item_move_action_node src/item_move_action_node.cpp)
ament_target_dependencies(item_move_action_node ${dependencies})

add_executable(item_victim_drop_action_node src/item_victim_drop_action_node.cpp)
ament_target_dependencies(item_victim_drop_action_node ${dependencies})

add_executable(item_victim_grab_action_node src/item_victim_grab_action_node.cpp)
ament_target_dependencies(item_victim_grab_action_node ${dependencies})

add_executable(obj_approach_action_node src/obj_approach_action_node.cpp)
ament_target_dependencies(obj_approach_action_node ${dependencies})

add_executable(target_approach_action_node src/target_approach_action_node.cpp)
ament_target_dependencies(target_approach_action_node ${dependencies})

add_executable(victim_rescue_action_node src/victim_rescue_action_node.cpp)
ament_target_dependencies(victim_rescue_action_node ${dependencies})

add_executable(explore_start_action_node src/explore_start_action_node.cpp)
ament_target_dependencies(explore_start_action_node ${dependencies})

add_executable(map_start_action_node src/map_start_action_node.cpp)
ament_target_dependencies(map_start_action_node ${dependencies})

add_executable(wander_around_action_node src/wander_around_action_node.cpp)
ament_target_dependencies(wander_around_action_node ${dependencies})

add_executable(obj_detect_start_action_node src/obj_detect_start_action_node.cpp)
ament_target_dependencies(obj_detect_start_action_node ${dependencies})

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node ${dependencies})

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
  obj_detect_start_action_node

  controller_node
 
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
