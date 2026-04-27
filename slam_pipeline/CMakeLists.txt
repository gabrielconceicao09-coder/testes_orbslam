cmake_minimum_required(VERSION 3.10)
project(slam_pipeline)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# =========================
# ROS2
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# =========================
# OpenCV
# =========================
find_package(OpenCV REQUIRED)
# =========================
# Eigen
# =========================
find_package(Eigen3 REQUIRED)

# =========================
# OpenGL + GLEW (CRÍTICO PARA ORB-SLAM3)
# =========================
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# =========================
# Includes ORB-SLAM3
# =========================
set(ORB_SLAM3_DIR /opt/ORB_SLAM3)

include_directories(
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/g2o

  /usr/include/eigen3
)

# =========================
# Pangolin (instalado no sistema)
# =========================
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# =========================
# Executável
# =========================
add_executable(orbslam_node
  src/orbslam_node.cpp
)

ament_target_dependencies(orbslam_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
)

# =========================
# LINKING (PARTE CRÍTICA)
# =========================
target_link_libraries(orbslam_node
  ${OpenCV_LIBS}

  /opt/ORB_SLAM3/lib/libORB_SLAM3.so

  /usr/local/lib/libpangolin.so

  Eigen3::Eigen

  OpenGL::GL
  GLEW::GLEW

  pthread
  dl
)

# =========================
# Install
# =========================
install(TARGETS orbslam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
