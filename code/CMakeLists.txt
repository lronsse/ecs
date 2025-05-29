cmake_minimum_required(VERSION 3.10)
project(CornerDetectionProject C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g") # Add warning and debug flags

# Explicitly list source files relative to CMAKE_CURRENT_SOURCE_DIR (which is code/)
set(SOURCES
    src/main.c
    src/csv_reader.c
    src/corner_detect.c
)

# Add the executable target
add_executable(cornerDetection ${SOURCES})

# Link the math library
target_link_libraries(cornerDetection PRIVATE m)

# Optional: Print a message after successful build
add_custom_target(BuildMessage ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Build complete. Executable is at ./cornerDetection"
    DEPENDS cornerDetection
)
