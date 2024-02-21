cmake_minimum_required(VERSION 3.0.0)
project(lab4)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include(CTest)
enable_testing()

add_executable(t1 t1.cpp)
target_link_libraries(t1 ${OpenCV_LIBS})
add_executable(t3 t3.cpp)
target_link_libraries(t3 ${OpenCV_LIBS})
add_executable(t4 t4.cpp)
target_link_libraries(t4 ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
include(CPack)
