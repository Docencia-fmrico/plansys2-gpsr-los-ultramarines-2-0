cmake_minimum_required(VERSION 3.5)
project(plansys2_gpsr)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_bt_actions
)

include_directories(include)

add_executable(gpsr_controller src/gpsr_controller.cpp)
ament_target_dependencies(gpsr_controller ${dependencies})

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

# add_library(bt_move SHARED src/bt_nodes/Move.cpp)
add_library(bt_pick SHARED src/nodes/PickItem.cpp)
add_library(bt_drop SHARED src/nodes/DropItem.cpp)
add_library(bt_closedoor SHARED src/nodes/CloseDoor.cpp)
add_library(bt_opendoor SHARED src/nodes/OpenDoor.cpp)


list(APPEND plugin_libs
  # bt_move
  bt_pick
  bt_drop
  bt_closedoor
  bt_opendoor
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(DIRECTORY launch pddl bt_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  gpsr_controller
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