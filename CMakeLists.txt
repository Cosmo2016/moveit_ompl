cmake_minimum_required(VERSION 2.8.3)
project(moveit_ompl)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(catkin REQUIRED COMPONENTS
  moveit_core 
  moveit_ros_planning
  moveit_visual_tools
  roscpp
  rosconsole
  pluginlib
  tf
  eigen_conversions
  ompl_thunder
)
find_package(OMPL REQUIRED)
#find_package(OpenMP)

catkin_package(
  LIBRARIES
    moveit_ompl
    ${OMPL_LIBRARIES}
  INCLUDE_DIRS
    include
    ${OMPL_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    ompl_thunder
)

include_directories(SYSTEM
                    ${Boost_INCLUDE_DIRS})

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${OMPL_INCLUDE_DIRS})

link_directories(${OMPL_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(MOVEIT_LIB_NAME moveit_ompl)

add_library(${MOVEIT_LIB_NAME}
  src/planning_context_manager.cpp
  src/model_based_planning_context.cpp
  src/parameterization/model_based_state_space.cpp
  src/parameterization/model_based_state_space_factory.cpp
  src/parameterization/joint_space/joint_model_state_space.cpp
  src/parameterization/joint_space/joint_model_state_space_factory.cpp  
  src/parameterization/humanoid_space/humanoid_model_state_space.cpp
  src/parameterization/humanoid_space/humanoid_model_state_space_factory.cpp
  src/parameterization/work_space/pose_model_state_space.cpp  
  src/parameterization/work_space/pose_model_state_space_factory.cpp
  src/detail/threadsafe_state_storage.cpp
  src/detail/state_validity_checker.cpp
  src/detail/projection_evaluators.cpp
  src/detail/goal_union.cpp
  src/detail/constrained_sampler.cpp
  src/detail/constrained_valid_state_sampler.cpp
  src/detail/constrained_goal_sampler.cpp
  src/detail/ompl_console.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

catkin_add_gtest(test_state_space test/test_state_space.cpp)
target_link_libraries(test_state_space ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#set_target_properties(test_state_space PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

add_library(moveit_ompl_planner_plugin src/ompl_planner_manager.cpp)
target_link_libraries(moveit_ompl_planner_plugin ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} moveit_ompl_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)


install(FILES moveit_ompl_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
