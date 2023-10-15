cmake_minimum_required(VERSION 3.5)

# Following directive defines the project name.
project(ros_gz_example_gazebo)
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
find_package(gz-sensors7 REQUIRED COMPONENTS)
set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})
find_package(gz-msgs10 REQUIRED COMPONENTS)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
find_package(gz-rendering7 REQUIRED COMPONENTS)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})
find_package(gz-transport12 REQUIRED COMPONENTS)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})


set(MSGS_PROTOS
   ${CMAKE_CURRENT_SOURCE_DIR}/proto/ros_gz_example_gazebo/msgs/obstaclerange.proto
)

gz_msgs_generate_messages(
  # The cmake target to be generated for libraries/executables to link
  TARGET msgs
  # The protobuf package to generate (Typically based on the path)
  PROTO_PACKAGE "ros_gz_example_gazebo.msgs"
  # The path to the base directory of the proto files
  # All import paths should be relative to this (eg gz/custom_msgs/vector3d.proto)
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
  # List of proto files to generate
  MSGS_PROTOS ${MSGS_PROTOS}
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)
add_library(msgs
  SHARED
  ${CMAKE_CURRENT_BINARY_DIR}/obstaclerange.pb.cc
)
target_include_directories(msgs PRIVATE include)

target_link_libraries(msgs gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER})

add_library(ObstacleRangeSensor
  SHARED
  src/ObstacleRangeSensor.cc
)

target_include_directories(
  ObstacleRangeSensor PRIVATE include
)

target_link_libraries(ObstacleRangeSensor PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
# gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
# gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
  ros_gz_example_gazebo-msgs

  )

# Following 'install' directive ensures that the compiled libraries
# of the 'BasicSystem' and 'ObstacleRangeSensor' targets will be copied
# to the subfolder 'lib/ros_gz_example_gazebo' of the install directory.
install(
  TARGETS 
  ObstacleRangeSensor 
  ros_gz_example_gazebo-msgs
  DESTINATION lib/${PROJECT_NAME}
)


# Following 'install' directive ensures that the 'worlds' subfolder
# will be copied to the 'share/ros_gz_example_gazebo/worlds'
# subfolder of the installation directory.
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)


# Following directives are used when testing.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'sourece install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
