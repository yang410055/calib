
set(SRC_LIST ${PROJECT_SOURCE_DIR}/src/main.cpp
    ${PROJECT_SOURCE_DIR}/src/computeHomograph.cpp
	${PROJECT_SOURCE_DIR}/src/computeV.cpp
	${PROJECT_SOURCE_DIR}/src/compute2cameraRt.cpp
	${PROJECT_SOURCE_DIR}/src/Refine.cpp
	${PROJECT_SOURCE_DIR}/src/ceres_refine.cpp
	  )
	  
message(${SRC_LIST})

#add_executable(basic_calib main.cpp)
add_executable(basic_calib ${SRC_LIST})

#####添加ceres相关库
set(CERES_LIBS D:/SRC_TestConfig/VS_C/ceres/ceres-solver/build/lib/Debug/ceres-debug.lib
              D:/SRC_TestConfig/VS_C/ceres/glog/build/Debug/glogd.lib
              dbghelp.lib
             D:/SRC_TestConfig/VS_C/ceres/gflags-master/build/lib/Debug/gflags_static_debug.lib
             shlwapi.lib
             kernel32.lib
             user32.lib
             gdi32.lib
             winspool.lib
             shell32.lib
             ole32.lib
             oleaut32.lib
             uuid.lib
             comdlg32.lib
             advapi32.lib)

target_link_libraries(basic_calib ${OpenCV_LIBS} ${CERES_LIBS})
#target_link_libraries(basic_calib ${OpenCV_LIBS})




message(STATUS ${OpenCV_LIBS})