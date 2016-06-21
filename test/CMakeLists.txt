cmake_minimum_required (VERSION 2.6)

project (BILHR)

#use c++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../build)

include_directories(../src)

add_executable(test_cmac test_cmac.cpp)
add_executable(test_mlp test_mlp.cpp)
add_executable(test_rl test_rl.cpp)