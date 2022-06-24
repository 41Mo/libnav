cmake_minimum_required(VERSION 3.10)
project(nav VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

option(TESTING "Enable testing" OFF)

add_compile_options(
    -O2
    -Wall
    -Werror=format-security
    -Warray-bounds
    -Wcast-align
    -Wcast-qual
    -Wconversion
    -Wctor-dtor-privacy
    -Wdisabled-optimization
    -Werror
    -Wextra
    -Wfloat-equal
    -Wformat-security
    -Wformat=2
    -Winit-self
    -Wlogical-op
    -Wmissing-include-dirs
    -Wno-sign-compare
    -Wno-unused
    -Wno-unused-parameter
    -Wnoexcept
    -Wold-style-cast
    -Woverloaded-virtual
    -Wpointer-arith
    -Wredundant-decls
    -Wreorder
    -Wshadow
    -Wsign-conversion
    -Wsign-promo
    -Wstrict-null-sentinel
    -Wswitch-default
    -Wundef
    -Wuninitialized
    -Wunused-variable
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1
    -D_GLIBCXX_ASSERTIONS
    -D_FORTIFY_SOURCE=2
    -fasynchronous-unwind-tables
    -fexceptions
    -fstack-clash-protection
    -fstack-protector-all
    -fstack-protector-strong
    -pipe
    -Wl,-z,defs
    -Wl,-z,now
    -Wl,-z,relro
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

add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)
if(TESTING)
    include_directories(
        include
        modules/Matrix/matrix
        interface
    )
    enable_testing()
    add_subdirectory(test)
    add_dependencies(check test_build)
endif()
