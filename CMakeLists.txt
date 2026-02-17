cmake_minimum_required(VERSION 3.15)

project(KinovaProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(JACO_SDK_DIR "C:/Program Files (x86)/JACO-SDK/API")

# First create executable
add_executable(kinova_app
    src/main.cpp
    src/move_to_point_angular_input.cpp
    src/kinova_ik.cpp
)

target_include_directories(kinova_app PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${JACO_SDK_DIR}
)


