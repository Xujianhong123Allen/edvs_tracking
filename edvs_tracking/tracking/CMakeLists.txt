cmake_minimum_required(VERSION 3.5)
project(tracking)

find_package(catkin_simple REQUIRED)
catkin_simple()
# Default to C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native -O3")

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(dvs_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package( G2O REQUIRED)
find_package( CSparse REQUIRED)
find_package(geometry_msgs REQUIRED)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
include_directories( ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
    "/usr/include/eigen3/"
)

add_executable(tracking src/tracking src/tracking_node.cpp)

target_link_libraries(tracking
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CSPARSE_LIBRARY}
  g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)

install(TARGETS
  tracking
  DESTINATION lib/${PROJECT_NAME}
)


