include(gtest.cmake)
link_libraries(nav)
set(tests
    test_input
    test_solution
    )

add_custom_target(test_build)
foreach(test_name ${tests})
    add_executable(${test_name}
        ${test_name}.cpp)
    add_test(test_${test_name} ${test_name})
    add_dependencies(test_build ${test_name})
endforeach()

# vim: set et fenc=utf-8 ft=cmake ff=unix sts=0 sw=4 ts=4 :
