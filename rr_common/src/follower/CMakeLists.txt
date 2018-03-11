add_executable(follower follower.cpp)
target_link_libraries(follower ${catkin_LIBRARIES})
add_dependencies(follower rr_platform_generate_messages_cpp)
