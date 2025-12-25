add_executable(read_test
    read.cpp
)
target_link_libraries(read_test PRIVATE moteus_api)
set_runtime_dir(read_test tests)

add_executable(write_test
    write.cpp
)
target_link_libraries(write_test PRIVATE moteus_api)
set_runtime_dir(write_test tests)
