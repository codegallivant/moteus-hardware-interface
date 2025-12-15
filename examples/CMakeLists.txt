add_executable(velocity
    velocity.cpp
)
target_link_libraries(velocity PRIVATE moteus_api)
set_runtime_dir(velocity examples)

add_executable(sample
    sample.cpp
)
target_link_libraries(sample PRIVATE moteus_api)
set_runtime_dir(sample examples)

add_executable(rezero
    rezero.cpp
)
target_link_libraries(rezero PRIVATE moteus_api)
set_runtime_dir(rezero examples)
