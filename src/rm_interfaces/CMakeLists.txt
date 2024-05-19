cmake_minimum_required(VERSION 3.10)
project(rm_decision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotHeat.msg"
  "msg/RobotStatus.msg"
  "msg/GameStatus.msg"
  "msg/Buffinfo.msg"
  "msg/JudgeSysShootHit.msg"
  "msg/BulletMove.msg"
  "msg/Position.msg"
  "msg/RobotPosition.msg"
  "msg/BattlePosition.msg"
  "msg/BulletsInfo.msg"

  "srv/BasicExecutor.srv"
  "srv/FricWhl.srv"
  "srv/GimbalAim.srv"
  "srv/GimbalMode.srv"
  "srv/JudgeSysControl.srv"
  "srv/ShootCmd.srv"

  "action/ArmorDetection.action"
  "action/GlobalPlanner.action"
  "action/LocalPlanner.action"
  
  

  DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    builtin_interfaces
)

ament_package()