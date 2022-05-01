cmake_minimum_required(VERSION 3.10)
project(libnav VERSION 0.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(
  -Wall
  -Wextra
  -Wconversion
  -Wsign-conversion
  $<$<CONFIG:DEBUG>:-g>
  $<$<CONFIG:DEBUG>:-O0>
  $<$<CONFIG:RELEASE>:-O3>
)

file(GLOB SRCS
    "src/*.cpp"
    "interface/*.cpp"
)

add_library(nav SHARED
    ${SRCS}
)

target_include_directories(nav PUBLIC
    include
    modules/Matrix/matrix
    INTERFACE interface
)