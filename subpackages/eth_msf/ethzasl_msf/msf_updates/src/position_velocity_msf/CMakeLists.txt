add_executable(position_velocity_sensor main.cpp)

target_link_libraries(position_velocity_sensor pose_distorter ${catkin_LIBRARIES})

add_dependencies(position_velocity_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS})
