cmake_minimum_required(VERSION 2.8.3)

project(duco_driver)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
  duco_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs roscpp std_msgs duco_msgs
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

include_directories(include ${catkin_INCLUDE_DIRS})

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        link_directories(${catkin_LIB_DIRS}  lib/lib64 lib/lib64/log4cplus lib/lib64/config lib/lib64/DucoCobotAPI)
else()
        link_directories(${catkin_LIB_DIRS}  lib/lib32)
endif()

## Declare a C++ executable
set(${PROJECT_NAME}_SOURCES1
    src/DucoDriverNode.cpp
    src/DucoDriver.cpp
)

add_executable(duco_driver ${${PROJECT_NAME}_SOURCES1})

target_link_libraries(duco_driver ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES}   DucoCobotAPI)


set(${PROJECT_NAME}_SOURCES2
    src/DucoRobotStatusNode.cpp
    src/DucoRobotStatus.cpp
)

add_executable(robot_status ${${PROJECT_NAME}_SOURCES2})

target_link_libraries(robot_status  ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES}   DucoCobotAPI)

#LINK_LIBRARIES(${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(CLIENT_SRC_FILES
  src/DucoRobotInterface.cpp
)

add_library(robot_interface ${CLIENT_SRC_FILES})
target_link_libraries(robot_interface   ${catkin_LIBRARIES} )
add_dependencies(robot_interface ${catkin_EXPORTED_TARGETS})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)


add_dependencies(duco_driver duco_msgs_gencpp)
add_dependencies(robot_status duco_msgs_gencpp)


