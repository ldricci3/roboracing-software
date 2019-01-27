find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers


add_executable(canny_line_detector canny_line_detector.cpp)
target_link_libraries(canny_line_detector ${catkin_LIBRARIES})
add_dependencies(canny_line_detector rr_platform_generate_messages_cpp)
target_link_libraries( canny_line_detector ${OpenCV_LIBS} )