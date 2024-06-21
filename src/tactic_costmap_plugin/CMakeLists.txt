cmake_minimum_required(VERSION 3.5)
project(tactic_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

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
# find_package(rclcpp REQUIRED)
# find_package(nav2_costmap_2d REQUIRED)
# find_package(pluginlib REQUIRED)

include_directories(include)

###########
## Build ##
###########

ament_auto_add_library(${lib_name} SHARED
            src/tactic_layer.cpp)

#############
## Install ##
#############

install(TARGETS ${lib_name}
        DESTINATION lib)

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

# === Ament work ===

# pluginlib_export_plugin_description_file() installs tactic_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d tactic_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()