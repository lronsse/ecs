cmake_minimum_required(VERSION 3.10)
project(cornerDetection C)

set(CMAKE_C_STANDARD 11)

# Set the path to the source directory
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

# List your source files with their relative path
set(SOURCES
    ${SRC_DIR}/main.c
    ${SRC_DIR}/csv_reader.c
    ${SRC_DIR}/corner_detect.c
)

add_executable(lidar_extraction ${SOURCES})
target_link_libraries(lidar_extraction m)

# Custom target to run the executable after building
add_custom_target(run
    COMMAND cornerDetection
    DEPENDS cornerDetection
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
