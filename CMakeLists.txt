cmake_minimum_required(VERSION 3.10)
project(rtask_commons)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

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

add_library(${PROJECT_NAME}_pddl_generator_lib src/pddl_generator/Helpers.cpp
                                               src/pddl_generator/LiteralTerm.cpp
                                               src/pddl_generator/NumericalTerm.cpp
                                               src/pddl_generator/NumericalFunction.cpp
                                               src/pddl_generator/NumericalOperator.cpp
                                               src/pddl_generator/LiteralExpression.cpp
                                               src/pddl_generator/NotExpression.cpp
                                               src/pddl_generator/AndExpression.cpp
                                               src/pddl_generator/OrExpression.cpp
                                               src/pddl_generator/WhenExpression.cpp
                                               src/pddl_generator/ExistsExpression.cpp
                                               src/pddl_generator/ForAllExpression.cpp
                                               src/pddl_generator/CompareExpression.cpp
                                               src/pddl_generator/ArithmeticExpression.cpp
                                               src/pddl_generator/ImplyExpression.cpp
                                               src/pddl_generator/EqualsExpression.cpp
                                               src/pddl_generator/Predicate.cpp
                                               src/pddl_generator/Action.cpp
                                               )

add_library(${PROJECT_NAME}_lib src/utils.cpp
                                src/property.cpp
                                src/capability.cpp
                                src/device.cpp
                                src/singleRoboticSystem.cpp
                                src/human.cpp
                                src/collectiveRoboticSystem.cpp
                                src/agent.cpp
                                src/agentGroup.cpp
                                src/typedTerm.cpp
#                                src/literalFormula.cpp
#                                src/predicate.cpp
#                                src/literalCondition.cpp
#                                src/action.cpp
#                                src/taskDefinition.cpp
#                                src/domain.cpp
#                                src/task.cpp
#                                src/mission.cpp
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
               LIBRARIES ${PROJECT_NAME}_lib ${PROJECT_NAME}_pddl_generator_lib)

# Add cmake target dependencies of the executable same as for the library above
add_dependencies(${PROJECT_NAME}_lib
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

set_target_properties(${PROJECT_NAME}_lib
                       PROPERTIES CXX_STANDARD_REQUIRED ON)

set_target_properties(${PROJECT_NAME}_lib PROPERTIES CXX_STANDARD 17)

target_include_directories(${PROJECT_NAME}_lib
                           PUBLIC include
                           PUBLIC ${catkin_INCLUDE_DIRS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_pddl_generator_lib ${catkin_LIBRARIES})

set_target_properties(${PROJECT_NAME}_pddl_generator_lib
                      PROPERTIES CXX_STANDARD_REQUIRED ON)

set_target_properties(${PROJECT_NAME}_pddl_generator_lib PROPERTIES CXX_STANDARD 17)

target_include_directories(${PROJECT_NAME}_pddl_generator_lib
                          PUBLIC include/pddl_generator
                          PUBLIC ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
 find_package(rostest REQUIRED)

 add_rostest_gtest(property_test launch/property_test.test test/property_test.cpp)
 target_link_libraries(property_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)

 add_rostest_gtest(capability_test launch/capability_test.test test/capability_test.cpp)
 target_link_libraries(capability_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)

 add_rostest_gtest(device_test launch/device_test.test test/device_test.cpp)
 target_link_libraries(device_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)

 add_rostest_gtest(single_robotic_system_test launch/single_robotic_system_test.test test/single_robotic_system_test.cpp)
 target_link_libraries(single_robotic_system_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)

 add_rostest_gtest(collective_robotic_system_test launch/collective_robotic_system_test.test test/collective_robotic_system_test.cpp)
 target_link_libraries(collective_robotic_system_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)

 add_rostest_gtest(human_test launch/human_test.test test/human_test.cpp)
 target_link_libraries(human_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)
 
 add_rostest_gtest(agent_test launch/agent_test.test test/agent_test.cpp)
 target_link_libraries(agent_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)

 add_rostest_gtest(agent_group_test launch/agent_group_test.test test/agent_group_test.cpp)
 target_link_libraries(agent_group_test ${catking_LIBRARIES} ${PROJECT_NAME}_lib)

endif()

add_executable(${PROJECT_NAME}_node
               test/main.cpp)

set_target_properties(${PROJECT_NAME}_node
                      PROPERTIES OUTPUT_NAME
                                 test
                                 PREFIX
                                 "")

add_dependencies(${PROJECT_NAME}_node
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
                      ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_pddl_generator_lib)

target_include_directories(${PROJECT_NAME}_node
                           PRIVATE include
                                   ${catkin_INCLUDE_DIRS})
