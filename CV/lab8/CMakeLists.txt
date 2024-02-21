cmake_minimum_required(VERSION 2.5)

project(lab8)

find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
              src/main.cpp
              src/function.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
