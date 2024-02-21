

cmake_minimum_required(VERSION 2.5)


project("CV-proj")

find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
								"src/main.cpp"
								"src/BoundingBox.cpp"
								"src/UmboFunct.cpp"
								"src/breadFunc.cpp"	
								"src/circles.cpp"
								"src/saladFunc.cpp"
								"src/foodSegmentation.cpp"
								"src/detection.cpp"
								"src/p_measure.cpp"
								"src/gt_reader.cpp"
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
