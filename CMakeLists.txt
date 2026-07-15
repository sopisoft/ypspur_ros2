cmake_minimum_required(VERSION 3.8...3.28)

project(yp_spur_ros2 VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

include(CTest)
include(CheckFunctionExists)
check_function_exists(longjmp HAVE_LONGJMP)
check_function_exists(siglongjmp HAVE_SIGLONGJMP)
check_function_exists(clock_nanosleep HAVE_CLOCK_NANOSLEEP)
check_function_exists(nanosleep HAVE_NANOSLEEP)

add_compile_definitions(
  PROJECT_VERSION=\"${PROJECT_VERSION}\"
  YP_PROTOCOL_NAME=\"YPP:11:05\"
  YP_PARAMS_DIR=\"robot-params\"
)

if(HAVE_LONGJMP)
  add_compile_definitions(HAVE_LONGJMP)
endif()
if(HAVE_SIGLONGJMP)
  add_compile_definitions(HAVE_SIGLONGJMP)
endif()
if(HAVE_CLOCK_NANOSLEEP)
  add_compile_definitions(HAVE_CLOCK_NANOSLEEP)
endif()
if(HAVE_NANOSLEEP)
  add_compile_definitions(HAVE_NANOSLEEP)
endif()

include_directories(
  include
  src
  vendor/yp-spur/include
  vendor/yp-spur/auxlib/include
  vendor/yp-spur/auxlib/formula-calc/include
)

add_subdirectory(vendor/yp-spur/auxlib)

set(YP_SPUR_CORE_SOURCES
  src/coordinator_adapter.c
  src/coordinator_device.c
  src/coordinator_runtime.c
  vendor/yp-spur/src/adinput.c
  vendor/yp-spur/src/command_run.c
  vendor/yp-spur/src/communication.c
  vendor/yp-spur/src/control_motion.c
  vendor/yp-spur/src/control_vehicle.c
  vendor/yp-spur/src/coordinator_runtime.c
  vendor/yp-spur/src/msq.win32.c
  vendor/yp-spur/src/odometry.c
  vendor/yp-spur/src/param.c
  vendor/yp-spur/src/serial.c
  vendor/yp-spur/src/ssm_spur_handler.c
  vendor/yp-spur/src/utility.c
  vendor/yp-spur/src/ypprotocol.c
  vendor/yp-spur/src/yprintf.c
)

add_library(ypspur_coordinator_core STATIC ${YP_SPUR_CORE_SOURCES})
target_compile_options(ypspur_coordinator_core PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ypspur_coordinator_core
  carte2d
  formula-calc
  m
  Threads::Threads
)

add_library(yp_spur_node_support STATIC
  src/node_util.cpp
  src/coordinator_client.cpp
  src/node_controller.cpp
)
target_compile_features(yp_spur_node_support PRIVATE cxx_std_17)
target_compile_options(yp_spur_node_support PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(yp_spur_node_support
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

add_executable(yp_spur_node
  src/yp_spur_node.cpp
)
target_compile_features(yp_spur_node PRIVATE cxx_std_17)
target_compile_options(yp_spur_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(yp_spur_node
  yp_spur_node_support
  ypspur_coordinator_core
)
ament_target_dependencies(yp_spur_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

if(BUILD_TESTING)
  add_executable(yp_spur_node_controller_test
    test/node_controller_ros2_test.cpp
  )
  target_compile_features(yp_spur_node_controller_test PRIVATE cxx_std_17)
  target_compile_options(yp_spur_node_controller_test PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(yp_spur_node_controller_test
    yp_spur_node_support
  )
  ament_target_dependencies(yp_spur_node_controller_test
    geometry_msgs
    nav_msgs
    rclcpp
    tf2_ros
  )
  add_test(NAME yp_spur_node_controller_test COMMAND yp_spur_node_controller_test)

  add_executable(yp_spur_node_util_test
    test/node_util_test.cpp
  )
  target_compile_features(yp_spur_node_util_test PRIVATE cxx_std_17)
  target_compile_options(yp_spur_node_util_test PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(yp_spur_node_util_test
    yp_spur_node_support
  )
  add_test(NAME yp_spur_node_util_test COMMAND yp_spur_node_util_test)
endif()

install(TARGETS
  ypspur_coordinator_core
  yp_spur_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
