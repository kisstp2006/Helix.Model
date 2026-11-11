cmake_minimum_required(VERSION 3.28)
project(Helix.Model LANGUAGES CXX)

if(NOT COMMAND helix_add_library)
    include("${CMAKE_CURRENT_SOURCE_DIR}/External/Helix.CMake/Helix.cmake")
endif()

helix_require(Core)
helix_require(Serialization)
helix_add_library(helix_model ALIAS Model PUBLIC_DEPS Helix::Core PRIVATE_DEPS Helix::Serialization)

helix_add_test_suite(helix_model_tests DEPS Helix::Model)
helix_add_benchmark_suite(helix_model_bench DEPS Helix::Model)
if(TARGET helix_model_bench)
    # The benchmarks build their models with the test writers.
    target_include_directories(helix_model_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Tests)
endif()
helix_add_fuzz_suite(helix_model_fuzz DEPS Helix::Model ROUNDS 1000)
