# rm_utils/CMakeLists.txt
# 创建静态库
add_library(PerformanceMonitor STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/performance_monitor.cpp
)

find_package(OpenCV REQUIRED)

# 设置库的包含目录
target_include_directories(PerformanceMonitor PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 链接所需库
target_link_libraries(PerformanceMonitor PUBLIC
    ${OpenCV_LIBS}
)

# 将库添加到全局变量中，供主CMakeLists.txt使用
list(APPEND MAIN_PROJECT_LIBRARIES PerformanceMonitor)
list(APPEND MAIN_PROJECT_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR})

set(MAIN_PROJECT_LIBRARIES ${MAIN_PROJECT_LIBRARIES} CACHE INTERNAL "Libraries for main project")
set(MAIN_PROJECT_INCLUDE_DIRS ${MAIN_PROJECT_INCLUDE_DIRS} CACHE INTERNAL "Include directories for main project")