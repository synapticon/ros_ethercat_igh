cmake_minimum_required(VERSION 3.5)
project(test_master)

set(CMAKE_CXX_STANDARD 14)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/control ${CMAKE_CURRENT_SOURCE_DIR}/logic)

add_subdirectory(control)
add_subdirectory(logic)

add_executable(test_master main.cpp ${test_master_src})
target_compile_options(test_master PUBLIC -DPB_FIELD_32BIT -DLOG_LEVEL_INFO -DENABLE_ASSERT)
target_link_libraries(test_master mcx-core)

install(TARGETS test_master DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_PREFIX}/share/test_master)