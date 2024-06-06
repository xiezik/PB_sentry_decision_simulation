
cmake_minimum_required(VERSION 3.10)
project(rm_costmap)


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

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
  proto/costmap_parameter_setting.proto
  proto/inflation_layer_setting.proto
  proto/obstacle_layer_setting.proto
  proto/static_layer_setting.proto
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${PROTO_SRCS}
  ${PROTO_HDRS}
  src/test_costmap.cpp
  src/costmap_2d.cpp
  src/costmap_interface.cpp
  src/costmap_layer.cpp
  src/costmap_math.cpp
  src/debuff_layer.cpp
  src/footprint.cpp
  src/inflation_layer.cpp
  src/layered_costmap.cpp
  src/observation_buffer.cpp
  src/obstacle_layer.cpp
  src/static_layer.cpp
  src/tactic_layer.cpp
  src/layer.cpp
)


##############
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

