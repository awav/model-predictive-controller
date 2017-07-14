project(mpc)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++14)

if(${CMAKE_BUILD_TYPE} MATCHES Release)
  add_definitions(-O3 -DNDEBUG)
else()
  add_definitions(-g)
endif()

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/mpc.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  include_directories(/usr/local/opt/openssl/include)
  link_directories(/usr/local/opt/openssl/lib)
  link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS)
