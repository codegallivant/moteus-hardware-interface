cmake_minimum_required(VERSION 3.10)

project(MoteusAPI VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

find_package(Threads REQUIRED)

add_executable(read
    tests/read.cpp
    src/moteus_api.cpp
)
target_link_libraries(read PRIVATE Threads::Threads)

add_executable(write
    tests/write.cpp
    src/moteus_api.cpp
)

# add_executable(load
#     scripts/load.cpp
    # includes/moteus/api/moteus_api.cpp
# )

add_executable(sample
    examples/sample.cpp
    src/moteus_api.cpp
)