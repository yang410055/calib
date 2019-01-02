
set(SRC_LIST ${PROJECT_SOURCE_DIR}/src/main.cpp
    ${PROJECT_SOURCE_DIR}/src/computeHomograph.cpp
	${PROJECT_SOURCE_DIR}/src/computeV.cpp
	${PROJECT_SOURCE_DIR}/src/compute2cameraRt.cpp
	${PROJECT_SOURCE_DIR}/src/Refine.cpp
	  )
	  
message(${SRC_LIST})

#add_executable(basic_calib main.cpp)
add_executable(basic_calib ${SRC_LIST})

target_link_libraries(basic_calib ${OpenCV_LIBS})
#target_link_libraries(basic_calib ${OpenCV_LIBS})
message(STATUS ${OpenCV_LIBS})