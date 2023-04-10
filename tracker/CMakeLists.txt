cmake_minimum_required(VERSION 3.0.2)
project(tracker)

## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             cmake_modules
            )
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

set(SRC_FILES src/main.cpp )

add_executable(tracker ${SRC_FILES})

target_link_libraries(tracker
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
                      ${boost_LIBRARIES}
                      )