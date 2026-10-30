cmake_minimum_required(VERSION 3.10)
project(simple_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    roslint
    controller_interface
    hardware_interface
    forward_command_controller
    pluginlib
    )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    LIBRARIES
    CATKIN_DEPENDS
    roscpp
    roslint
    controller_interface
    hardware_interface
    forward_command_controller
    pluginlib
    DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    # Set manually because Eigen sets a non standard INCLUDE DIR
)

## Declare cpp executables
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(
#    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
## Mark cpp header files for installation
#install(
#    DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.hpp"
#)
#
## Mark other files for installation
#install(
#    DIRECTORY doc
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#
##############
### Testing ##
##############
#
#if (${CATKIN_ENABLE_TESTING})
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#  ## Add gtest based cpp test target and link libraries
#  catkin_add_gtest(${PROJECT_NAME}-test
#      test/test_ros_package_template.cpp
#      test/AlgorithmTest.cpp)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif ()

##########################
## Static code analysis ##
##########################

roslint_cpp()

add_library(simple_chassis_controller_plugins
        src/simple_chassis_controller.cpp
        src/Mecanum_odom.cpp
)
target_link_libraries(simple_chassis_controller_plugins ${catkin_LIBRARIES})
add_dependencies(simple_chassis_controller_plugins ${catkin_EXPORTED_TARGETS})

install(TARGETS simple_chassis_controller_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES simple_chassis_controller_plugins_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 发送
add_executable(chassis_cmd_handle src/chassis_cmd_handle.cpp
)
target_include_directories( chassis_cmd_handle PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)
add_dependencies(chassis_cmd_handle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(chassis_cmd_handle
        ${catkin_LIBRARIES}
)


#线性代收库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})