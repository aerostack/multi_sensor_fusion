add_executable(pose_velocity_sensor main.cpp)

target_link_libraries(pose_velocity_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(pose_velocity_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
