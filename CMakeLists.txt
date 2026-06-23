cmake_minimum_required(VERSION 3.16)
project(sbr LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC: force UTF-8 source encoding (fixes C4819 on CJK systems)
if(MSVC)
    add_compile_options(/utf-8)
endif()

# ── 头文件路径 ──
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/external/doctest)

# ── sbr_core 静态库 ──
add_library(sbr_core STATIC
    src/sbr_accelerator_bruteforce.cpp
    src/sbr_bvh_accelerator.cpp
    src/sbr_material.cpp
    src/sbr_scene_builder.cpp
    src/sbr_engine.cpp
    src/sbr_ray_emitter.cpp
    src/sbr_scene_loader.cpp
)

# ── sbr_app 独立可执行文件 ──
add_executable(sbr_app
    app/main.cpp
)
target_link_libraries(sbr_app PRIVATE sbr_core)
if(OpenMP_CXX_FOUND)
    target_link_libraries(sbr_app PRIVATE OpenMP::OpenMP_CXX)
endif()

# ── sbr_tests 测试可执行文件 (快速) ──
add_executable(sbr_tests_quick
    test/test_main.cpp
    test/test_math.cpp
    test/test_bruteforce.cpp
    test/test_scene_builder.cpp
    test/test_type_compat.cpp
    test/test_bvh_quick.cpp
    test/benchmark_perf.cpp
)
target_link_libraries(sbr_tests_quick PRIVATE sbr_core)
if(OpenMP_CXX_FOUND)
    target_link_libraries(sbr_tests_quick PRIVATE OpenMP::OpenMP_CXX)
endif()

# ── sbr_tests 完整测试套件 ──
add_executable(sbr_tests
    test/test_main.cpp
    test/test_math.cpp
    test/test_bruteforce.cpp
    test/test_scene_builder.cpp
    test/smoke_los.cpp
    test/smoke_pec_reflection.cpp
    test/smoke_transmission.cpp
    test/smoke_diffraction.cpp
    test/smoke_postprocess.cpp
    test/benchmark_perf.cpp
)
target_link_libraries(sbr_tests PRIVATE sbr_core)

# ── OpenMP 并行 (Phase 5) ──
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(sbr_core PRIVATE OpenMP::OpenMP_CXX)
    target_link_libraries(sbr_tests PRIVATE OpenMP::OpenMP_CXX)
    target_link_libraries(sbr_tests_quick PRIVATE OpenMP::OpenMP_CXX)
endif()
