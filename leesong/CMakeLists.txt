cmake_minimum_required(VERSION 3.10)
set(PACKAGE_NAME lane_detection)
project(${PACKAGE_NAME} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(src)

# Declare the executable target built from your sources
add_executable(main src/main.cpp)

# 라이브러리를 링킹해주기
target_link_libraries(main PRIVATE src)
