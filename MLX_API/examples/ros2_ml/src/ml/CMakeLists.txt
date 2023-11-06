cmake_minimum_required(VERSION 3.5)
project(ml)

## SOSLAB Libraries
execute_process( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

set( SOSLAB_LIBRARIES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libsoslab" )
set( SOSLAB_LIBRARIES "${SOSLAB_LIBRARIES_DIR}/libsoslab_core.so" "${SOSLAB_LIBRARIES_DIR}/libsoslab_ml.so" )

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(cv_bridge-config REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.8 REQUIRED)


include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${SOSLAB_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}_node 
    rclcpp
    sensor_msgs
    cv_bridge
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${SOSLAB_LIBRARIES} DESTINATION lib) 

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


#file(COPY ${SOSLAB_LIBRARIES} DESTINATION ${CATKIN_DEVEL_PREFIX}/lib)


ament_package()
