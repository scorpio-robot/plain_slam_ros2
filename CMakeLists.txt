cmake_minimum_required(VERSION 3.11)
project(plain_slam_ros2)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(Sophus QUIET)
find_package(nanoflann QUIET)

if(NOT Sophus_FOUND)
  message(STATUS "System-installed Sophus not found. Fetching Sophus via FetchContent...")

  include(FetchContent)
  FetchContent_Declare(
    Sophus
    GIT_REPOSITORY https://github.com/strasdat/Sophus.git
    GIT_TAG 1.22.10
  )

  FetchContent_MakeAvailable(Sophus)
endif()


if(NOT nanoflann_FOUND OR nanoflann_VERSION VERSION_LESS_EQUAL "1.6.0")
  message(STATUS "System-installed nanoflann version <= 1.6.0 or not found. Fetching nanoflann v1.8.0 via FetchContent...")

  include(FetchContent)
  FetchContent_Declare(
    nanoflann
    GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
    GIT_TAG v1.8.0
  )

  FetchContent_Populate(nanoflann)
  # Since nanoflann is header-only, we just need to include its headers
  set(nanoflann_INCLUDE_DIRS ${nanoflann_SOURCE_DIR}/include)
endif()

ament_auto_find_build_dependencies()

###########
## Build ##
###########

# lio_3d_node (as component library)
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${nanoflann_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} yaml-cpp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "plain_slam::LIO3DNode"
  EXECUTABLE lio_3d_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "plain_slam::SLAM3DNode"
  EXECUTABLE slam_3d_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "plain_slam::PC2ToPCDNode"
  EXECUTABLE pc2_to_pcd_node
)

# lidar_lidar_calibration_node
ament_auto_add_executable(lidar_lidar_calibration_node
  src/lidar_lidar_calibration_node.cpp
)

target_link_libraries(lidar_lidar_calibration_node ${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
