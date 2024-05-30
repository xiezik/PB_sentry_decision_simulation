
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

###########
## Build ##
###########

# proto file
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  CostmapProtoSrcs
  CostmapProtoHeaders
  ${ProtoFiles}
  )

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${DecisionProtoSrc}
  
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN JudgesysControl::JudgesysControl
  EXECUTABLE ${PROJECT_NAME}_node
)

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

