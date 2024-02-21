cmake_minimum_required(VERSION 2.5)

project(lab6_211316)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
              main.cpp
              function.h
              function.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
