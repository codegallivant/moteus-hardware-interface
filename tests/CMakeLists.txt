add_executable(read
    read.cpp
)
target_link_libraries(read PRIVATE moteus_api)
set_runtime_dir(read tests)

add_executable(write
    write.cpp
)
target_link_libraries(write PRIVATE moteus_api)
set_runtime_dir(write tests)
