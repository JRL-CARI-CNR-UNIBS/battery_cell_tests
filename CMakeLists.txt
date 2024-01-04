cmake_minimum_required(VERSION 3.0.2)
project(battery_cell_tests)
add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
## common

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  behaviortree_cpp
  skills_util
  skills_executer_msgs
  skills_learning_msgs
  skills_arbitrator_msgs
  skills_util_msgs
  manipulation_msgs
  actionlib_msgs
  actionlib
  pybullet_utils
  tf
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   SetMongoByParam.srv
#   SetParamByMongo.srv
#   RLParamsWrite.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

catkin_package(
 INCLUDE_DIRS
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS   roscpp
                  rospy
                  std_msgs
                  behaviortree_cpp
                  manipulation_msgs
                  actionlib_msgs
                  actionlib
                  gazebo_msgs
                  skills_util
                  skills_executer_msgs
                  skills_learning_msgs
                  skills_arbitrator_msgs
                  skills_util_msgs
                  pybullet_utils
                  tf
DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(        coke_tests src/coke_experiments/coke_tests.cpp)
# add_dependencies(      coke_tests ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries( coke_tests
#         ${catkin_LIBRARIES}
# )
