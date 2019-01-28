find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers


add_executable(canny_line_detector canny_line_detector.cpp)
target_link_libraries(canny_line_detector ${catkin_LIBRARIES})
target_link_libraries( canny_line_detector ${OpenCV_LIBS} )
target_link_libraries( canny_line_detector ${OpenCV_LIBS} )

add_executable(canny_editor canny_editor.cpp)
target_link_libraries(canny_editor ${catkin_LIBRARIES})
target_link_libraries( canny_editor ${OpenCV_LIBS} )
add_dependencies(canny_editor rr_platform_generate_messages_cpp)

