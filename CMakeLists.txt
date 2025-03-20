cmake_minimum_required(VERSION 3.8)
project(reachability_map_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rclcpp
  visualization_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_subdirectory(bonxai_core)

set(TARGET_NAME reachability_map_moveit)

add_executable(${TARGET_NAME} src/reachability_map_moveit.cpp)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  bonxai_core/include
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(${TARGET_NAME} ${PCL_LIBRARIES})

install(TARGETS ${TARGET_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


ament_package()
