# Core library: portable C++17, no OS or framework headers.

# The parameter header is generated from config/robot.yaml (needs Python with pyyaml).
set(CATENA_PYTHON "" CACHE FILEPATH "Python with pyyaml, used to generate the parameter header")
if(NOT CATENA_PYTHON)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(CATENA_PYTHON "${Python3_EXECUTABLE}")
endif()

set(CATENA_GEN_DIR "${CMAKE_BINARY_DIR}/generated")
set(CATENA_PARAMS_H "${CATENA_GEN_DIR}/catena/params_gen.h")
add_custom_command(
  OUTPUT "${CATENA_PARAMS_H}"
  COMMAND "${CMAKE_COMMAND}" -E env "PYTHONPATH=${CMAKE_SOURCE_DIR}"
          "${CATENA_PYTHON}" -m sim.config --header "${CATENA_PARAMS_H}"
  DEPENDS "${CMAKE_SOURCE_DIR}/config/robot.yaml" "${CMAKE_SOURCE_DIR}/sim/config.py"
  COMMENT "Generating params_gen.h from config/robot.yaml"
  VERBATIM)
add_custom_target(catena_params DEPENDS "${CATENA_PARAMS_H}")

add_library(catena_obj OBJECT
  src/catena_c.cpp
  src/chain.cpp
  src/ranging.cpp
  src/robot.cpp)
add_dependencies(catena_obj catena_params)
target_include_directories(catena_obj PUBLIC include "${CATENA_GEN_DIR}")
target_compile_options(catena_obj PRIVATE -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti)
set_target_properties(catena_obj PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON)

# Shared library for the simulator (ctypes); static library for the unit tests.
add_library(catena SHARED $<TARGET_OBJECTS:catena_obj>)
add_library(catena_static STATIC $<TARGET_OBJECTS:catena_obj>)
target_include_directories(catena_static PUBLIC include src "${CATENA_GEN_DIR}")
