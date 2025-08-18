# 创建静态库
add_library(armor_detector STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/armor_detector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/armor_detector_debug.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/light_corner_correct.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/number_classifier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/thread/armor_detector_thread.cpp
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 设置库的包含目录
target_include_directories(armor_detector PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接所需库
target_link_libraries(armor_detector PUBLIC
    ${OpenCV_LIBS}
    pubsub
    PerformanceMonitor
    serial
)

# 将库添加到全局变量中，供主CMakeLists.txt使用
list(APPEND MAIN_PROJECT_LIBRARIES armor_detector)
list(APPEND MAIN_PROJECT_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(MAIN_PROJECT_LIBRARIES ${MAIN_PROJECT_LIBRARIES} CACHE INTERNAL "Libraries for main project")
set(MAIN_PROJECT_INCLUDE_DIRS ${MAIN_PROJECT_INCLUDE_DIRS} CACHE INTERNAL "Include directories for main project")