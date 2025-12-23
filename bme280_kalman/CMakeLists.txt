cmake_minimum_required(VERSION 3.10)
project(bme280_kalman C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(SOURCES
    src/main.c
    src/kalman.c
    src/bme280.c
)

set(HEADERS
    src/kalman.h
    src/bme280.h
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE src)

target_link_libraries(${PROJECT_NAME} PRIVATE m)

target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -O2
)
