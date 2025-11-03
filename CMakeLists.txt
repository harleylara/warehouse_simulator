cmake_minimum_required(VERSION 3.8)
project(warehouse_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- SIM (Fortress) ---
find_package(gz-sim6 QUIET)
if(NOT gz-sim6_FOUND)
  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_TARGET ignition-gazebo6::ignition-gazebo6)
else()
  set(GZ_SIM_TARGET gz-sim6::gz-sim6)
endif()

# --- MATH ---
find_package(gz-math7 QUIET)
if(NOT gz-math7_FOUND)
  # En Fortress suele ser ignition-math6
  find_package(ignition-math6 REQUIRED)
  set(GZ_MATH_TARGET ignition-math6::ignition-math6)
else()
  set(GZ_MATH_TARGET gz-math7::gz-math7)
endif()

# --- PLUGIN ---
find_package(ignition-plugin1 QUIET COMPONENTS register)
if(NOT ignition-plugin1_FOUND)
  # (Sólo si alguna vez migras a Garden/Harmonic con gz-plugin2)
  find_package(gz-plugin2 REQUIRED COMPONENTS register)
  set(GZ_PLUGIN_TARGET gz-plugin2::gz-plugin2)
else()
  set(GZ_PLUGIN_TARGET ignition-plugin1::ignition-plugin1)
endif()

# Instalar carpetas de recursos
foreach(dir launch  worlds config)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(
      DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir}
      USE_SOURCE_PERMISSIONS
    )
  endif()
endforeach()

add_library(RandomActorWalkingPlugin SHARED src/RandomActorWalkingPlugin.cpp)
target_link_libraries(RandomActorWalkingPlugin
  PRIVATE ${GZ_SIM_TARGET}
          ${GZ_PLUGIN_TARGET}
          ${GZ_MATH_TARGET})
install(TARGETS RandomActorWalkingPlugin
        LIBRARY DESTINATION lib)

add_library(ActorWaypointFollowerPlugin SHARED src/ActorWaypointFollowerPlugin.cpp)
target_link_libraries(ActorWaypointFollowerPlugin
  PRIVATE ${GZ_SIM_TARGET} ${GZ_PLUGIN_TARGET} ${GZ_MATH_TARGET} yaml-cpp)
install(TARGETS ActorWaypointFollowerPlugin
        LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
