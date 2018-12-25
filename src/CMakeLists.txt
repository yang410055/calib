
add_executable(basic_calib main.cpp)
target_link_libraries(basic_calib ${OpenCV_LIBS})
message(STATUS ${OpenCV_LIBS})