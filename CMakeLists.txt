cmake_minimum_required(VERSION 2.8.3)
project(rtask_commons)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS roscpp
                        rtask_msgs)

catkin_package(CATKIN_DEPENDS roscpp
                              rtask_msgs
                              message_runtime)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include
                     ${catkin_INCLUDE_DIRS}
                     )

# Display additional files in qtcreator
file(GLOB_RECURSE
     FILES_LIST
     "*.msg"
     "*.srv"
     "*.xml")

add_custom_target(additional_files_${PROJECT_NAME} SOURCES ${FILES_LIST})

# Declare a C++ executable With catkin_make all packages are built within a
# single CMake context The recommended prefix ensures that target names across
# packages don't collide add_executable(${PROJECT_NAME}_node
add_library(${PROJECT_NAME}_lib src/utils.cpp
                                src/agent.cpp
                                src/capacity.cpp
                                src/component.cpp
                                src/property.cpp
                                src/status.cpp
                                src/action.cpp
                                src/taskDefinition.cpp
                                src/domain.cpp
                                src/task.cpp
                                #src/mission.cpp
                                )

# Rename C++ executable without prefix The above recommended prefix causes long
# target names, the following renames the target back to the shorter version for
# ease of user use e.g. "rosrun someones_pkg node" instead of "rosrun
# someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_lib
#                       PROPERTIES LIBRARY_OUTPUT_NAME
#                                  ${PROJECT_NAME}
#                                  PREFIX
#                                  "lib")

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}_lib)

# Add cmake target dependencies of the executable same as for the library above
add_dependencies(${PROJECT_NAME}_lib
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

#set_target_properties(${PROJECT_NAME}_lib
#                       PROPERTIES PUBLIC_HEADER include)

target_include_directories(${PROJECT_NAME}_lib
                           PUBLIC include
                           PRIVATE ${catkin_INCLUDE_DIRS})

#if(CATKIN_ENABLE_TESTING)
# find_package(rostest REQUIRED)
 
# add_rostest_gtest(capacity_test launch/capacity_test.test test/capacity_test.cpp)
# target_link_libraries(capacity_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)

# add_rostest_gtest(property_test launch/property_test.test test/property_test.cpp)
# target_link_libraries(property_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)

# add_rostest_gtest(component_test launch/component_test.test test/component_test.cpp)
# target_link_libraries(component_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)

# add_rostest_gtest(agent_test launch/agent_test.test test/agent_test.cpp)
# target_link_libraries(agent_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)

#endif()
