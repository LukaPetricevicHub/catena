# C++ unit tests for core (doctest, vendored in third_party/).
add_executable(catena_tests
  main.cpp
  test_params.cpp)
add_dependencies(catena_tests catena_params)
target_include_directories(catena_tests PRIVATE third_party)
target_link_libraries(catena_tests PRIVATE catena_static)
add_test(NAME core_unit_tests COMMAND catena_tests)
