cmake_minimum_required(VERSION 3.10)
project(CornerDetectionProject C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g")

find_package(Python3 COMPONENTS Interpreter)

set(SOURCES
    src/main.c
    src/csv_reader.c
    src/corner_detect.c
)

add_executable(cornerDetection ${SOURCES})
target_link_libraries(cornerDetection PRIVATE m)

add_custom_target(run_and_plot
    COMMAND ${CMAKE_BINARY_DIR}/cornerDetection  # Runs cornerDetection first
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/data # Create data directory
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_BINARY_DIR}/lidarDataCartesian.csv
            ${CMAKE_SOURCE_DIR}/data/lidarDataCartesian.csv # Copy to data/
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_BINARY_DIR}/outputCorners.csv
            ${CMAKE_SOURCE_DIR}/data/outputCorners.csv # Copy to data/
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/plotting.py # plotting.py will be updated to read from data/
    DEPENDS cornerDetection
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running cornerDetection, creating data/ dir, copying CSVs to data/, and then launching plotting.py"
)
