cmake_minimum_required(VERSION 3.5)
project(controller)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(charge_action_node src/charge_action_node.cpp)
ament_target_dependencies(charge_action_node ${dependencies})

add_executable(ask_charge_action_node src/ask_charge_action_node.cpp)
ament_target_dependencies(ask_charge_action_node ${dependencies})

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node ${dependencies})

add_executable(approach_balls_action_node src/approach_balls_action_node.cpp)
ament_target_dependencies(approach_balls_action_node ${dependencies})

add_executable(approach_balls_target_action_node src/approach_balls_target_action_node.cpp)
ament_target_dependencies(approach_balls_target_action_node ${dependencies})

add_executable(drop_balls_action_node src/drop_balls_action_node.cpp)
ament_target_dependencies(drop_balls_action_node ${dependencies})

add_executable(explore_action_node src/explore_action_node.cpp)
ament_target_dependencies(explore_action_node ${dependencies})

add_executable(grab_balls_action_node src/grab_balls_action_node.cpp)
ament_target_dependencies(grab_balls_action_node ${dependencies})

add_executable(handle_balls_action_node src/handle_balls_action_node.cpp)
ament_target_dependencies(handle_balls_action_node ${dependencies})

add_executable(map_action_node src/map_action_node.cpp)
ament_target_dependencies(map_action_node ${dependencies})

add_executable(wander_action_node src/wander_action_node.cpp)
ament_target_dependencies(wander_action_node ${dependencies})

add_executable(action_controller src/action_controller.cpp)
ament_target_dependencies(action_controller ${dependencies})


install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  controller_node
  move_action_node
  charge_action_node

  ask_charge_action_node
  approach_balls_action_node
  approach_balls_target_action_node

  drop_balls_action_node
  explore_action_node
  grab_balls_action_node

  handle_balls_action_node
  map_action_node
  wander_action_node

  action_controller
  
  

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
