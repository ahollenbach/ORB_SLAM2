cmake_minimum_required(VERSION 2.8.3)

project(ORB_SLAM2)
find_package(catkin REQUIRED COMPONENTS
     roscpp
     sensor_msgs
     image_transport
     message_filters
     cv_bridge
     tf
     cmake_modules)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(octomap REQUIRED)

catkin_package()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
${OCTOMAP_INCLUDE_DIRS}
)

set(LIBS
${catkin_LIBRARIES}
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${OCTOMAP_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
)

# Node for monocular camera
add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)

# Node for monocular multiple
add_executable(MonoMultiple
src/ros_mono_multiple.cc
)

target_link_libraries(MonoMultiple
${LIBS}
)

# Node for monocular server
add_executable(MonoServer
src/mono_multiple_server.cc
)

target_link_libraries(MonoServer
${LIBS}
)

# Node for stereo camera
add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

