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

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node ${dependencies})

add_executable(approach_items_action_node src/approach_items_action_node.cpp)
ament_target_dependencies(approach_items_action_node ${dependencies})

add_executable(approach_items_target_action_node src/approach_items_target_action_node.cpp)
ament_target_dependencies(approach_items_target_action_node ${dependencies})

add_executable(drop_items_action_node src/drop_items_action_node.cpp)
ament_target_dependencies(drop_items_action_node ${dependencies})


add_executable(grab_items_action_node src/grab_items_action_node.cpp)
ament_target_dependencies(grab_items_action_node ${dependencies})

add_executable(handle_items_action_node src/handle_items_action_node.cpp)
ament_target_dependencies(handle_items_action_node ${dependencies})

add_executable(action_controller src/action_controller.cpp)
ament_target_dependencies(action_controller ${dependencies})

add_executable(camera_active_action_node src/camera_active_action_node.cpp)
ament_target_dependencies(camera_active_action_node ${dependencies})

add_executable(battery_charge_1_action_node src/battery_charge_1_action_node.cpp)
ament_target_dependencies(battery_charge_1_action_node ${dependencies})

add_executable(charger_approach_action_node src/charger_approach_action_node.cpp)
ament_target_dependencies(charger_approach_action_node ${dependencies})

add_executable(explore_start_action_node src/explore_start_action_node.cpp)
ament_target_dependencies(explore_start_action_node ${dependencies})

add_executable(map_start_action_node src/map_start_action_node.cpp)
ament_target_dependencies(map_start_action_node ${dependencies})

add_executable(wander_around_action_node src/wander_around_action_node.cpp)
ament_target_dependencies(wander_around_action_node ${dependencies})

add_executable(obj_detect_start_action_node src/obj_detect_start_action_node.cpp)
ament_target_dependencies(obj_detect_start_action_node ${dependencies})



install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS

  action_controller
  approach_items_action_node
  approach_items_target_action_node

  battery_charge_1_action_node
  camera_active_action_node
  charger_approach_action_node

  controller_node
  drop_items_action_node
  explore_start_action_node

  grab_items_action_node
  handle_items_action_node
  map_start_action_node

  obj_detect_start_action_node
  wander_around_action_node


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
