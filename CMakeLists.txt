cmake_minimum_required(VERSION 2.8.3)
project(mesh_to_traversability_image)
set(CMAKE_BUILD_TYPE RELEASE)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_package(INCLUDE_DIRS include)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})

cs_add_library(mesh_to_traversability_image
  src/library/mesh_to_traversability_image.cpp
)

# create node to load mesh and create traversability from it
cs_add_executable(mesh_to_traversability_image_node
  src/mesh_to_traversability_image_node.cpp
  include/mesh_to_traversability_image/mesh_to_traversability_image.hpp
)
target_link_libraries(mesh_to_traversability_image_node mesh_to_traversability_image ${catkin_LIBRARIES})

cs_install()
cs_export()
