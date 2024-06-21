cmake_minimum_required(VERSION 3.10)
project(rm_decision)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


###########
## Build ##
###########

# protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/decision.proto)
# protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS EXPORT_MACRO DLL_EXPORT proto/decision.proto)
# protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS DESCRIPTORS PROTO_DESCS proto/decision.proto)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${PROTO_SRCS}
  ${PROTO_HDRS}
  executor/chassis_executor.cpp
  executor/gimbal_executor.cpp
  executor/point_navigation.cpp
  simulation/basic_executor.cpp
  simulation/collective_decision.cpp
  behavior_test.cpp
)

# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN JudgesysControl::JudgesysControl
#   EXECUTABLE ${PROJECT_NAME}_node
# )

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)