# rm_utils/CMakeLists.txt
# 创建静态库
add_library(pubsub STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/pubsub.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/thread/pubsub_thread.cpp
)

find_package(OpenCV REQUIRED)

# 设置库的包含目录
target_include_directories(pubsub PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 链接所需库
target_link_libraries(pubsub PUBLIC
    PerformanceMonitor
)

# 将库添加到全局变量中，供主CMakeLists.txt使用
list(APPEND MAIN_PROJECT_LIBRARIES pubsub)
list(APPEND MAIN_PROJECT_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR})

set(MAIN_PROJECT_LIBRARIES ${MAIN_PROJECT_LIBRARIES} CACHE INTERNAL "Libraries for main project")
set(MAIN_PROJECT_INCLUDE_DIRS ${MAIN_PROJECT_INCLUDE_DIRS} CACHE INTERNAL "Include directories for main project")

# 添加性能测试可执行文件
add_executable(pubsub_performance_test ${CMAKE_CURRENT_SOURCE_DIR}/test/test.cpp)
target_link_libraries(pubsub_performance_test pubsub ${OpenCV_LIBS})
# 设置库的包含目录
target_include_directories(pubsub_performance_test PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)