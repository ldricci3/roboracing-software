add_executable(image_transform image_transform.cpp)
target_link_libraries(image_transform ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(image_transform rr_platform_generate_messages_cpp)
