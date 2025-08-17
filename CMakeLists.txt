cmake_minimum_required(VERSION 3.10)
project(base LANGUAGES CXX)

# C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1) 找 OpenCV
find_package(OpenCV REQUIRED)
# 找fmt库
find_package(fmt REQUIRED)

# 2) 手动指定 MVS 路径（安装默认 /opt/MVS）
set(MVS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/hikcamera/SDK")
set(MVS_INCLUDE_DIR "${MVS_ROOT}/include")
set(MVS_LIB_DIR     "${MVS_ROOT}/lib/64")

# 3) 添加可执行文件
add_executable(${PROJECT_NAME}
        main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/hikcamera/src/HikCamera.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/thread/cam_thread.cpp
)



# 4) 包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${MVS_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/hikcamera/include
        ${CMAKE_CURRENT_SOURCE_DIR}/serial
        ${CMAKE_CURRENT_SOURCE_DIR}/armor_detector
)


# 5) 链接库
target_link_directories(${PROJECT_NAME} PRIVATE ${MVS_LIB_DIR})
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        MvCameraControl
        fmt::fmt  # 链接fmt库
)


# 6) 运行时 rpath，防止找不到 .so
target_link_options(${PROJECT_NAME} PRIVATE
        "-Wl,-rpath,${MVS_LIB_DIR}"
)


# 创建软链接到源代码中的config文件夹
if(UNIX AND NOT APPLE)
    # Linux系统使用create_symlink
    execute_process(
            COMMAND ${CMAKE_COMMAND} -E create_symlink
            ${CMAKE_CURRENT_SOURCE_DIR}/config
            ${CMAKE_CURRENT_BINARY_DIR}/config
    )
elseif(APPLE)
    # macOS系统也使用create_symlink
    execute_process(
            COMMAND ${CMAKE_COMMAND} -E create_symlink
            ${CMAKE_CURRENT_SOURCE_DIR}/config
            ${CMAKE_CURRENT_BINARY_DIR}/config
    )
else()
    # Windows系统回退到复制方式
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()
