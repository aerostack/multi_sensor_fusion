add_executable(pose_velocity_altitude_sensor main.cpp)

target_link_libraries(pose_velocity_altitude_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(pose_velocity_altitude_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
