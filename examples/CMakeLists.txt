add_executable(velocity_example
    velocity.cpp
)
target_link_libraries(velocity_example PRIVATE moteus_api)
set_runtime_dir(velocity_example examples)

add_executable(sample_example
    sample.cpp
)
target_link_libraries(sample_example PRIVATE moteus_api)
set_runtime_dir(sample_example examples)

add_executable(rezero_example
    rezero.cpp
)
target_link_libraries(rezero_example PRIVATE moteus_api)
set_runtime_dir(rezero_example examples)

add_executable(read_example
    read.cpp
)
target_link_libraries(read_example PRIVATE moteus_api)
set_runtime_dir(read_example examples)
