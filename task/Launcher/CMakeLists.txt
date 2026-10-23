cmake_minimum_required(VERSION 3.10)

project(launcher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(TASK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../.. ABSOLUTE)

# Output to app/ directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../app/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../app/)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../app/)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../app/)

# Link directories (must be BEFORE add_executable)
link_directories(${TASK_ROOT_DIR}/bbt_task_lib/)
link_directories(${TASK_ROOT_DIR}/bbt_base_lib/)

# Launcher sources
add_executable(taskframe_launcher
    src/main.cpp
    src/launcher.cpp
    src/worker_manager.cpp
    src/config_parser.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/ipc_protocol.cpp
)

# Include paths: Launcher own headers + TaskFrame headers + BBeamTech base libs
target_include_directories(taskframe_launcher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TASK_ROOT_DIR}/bbt_task_include/taskframe/
    ${TASK_ROOT_DIR}/bbt_task_include/worker_base/
    ${TASK_ROOT_DIR}/bbt_task_include/
    ${TASK_ROOT_DIR}/bbt_base_include/
    ${TASK_ROOT_DIR}/bbt_base_include/osbase/
    ${TASK_ROOT_DIR}/bbt_base_include/confbase/
    ${TASK_ROOT_DIR}/bbt_base_include/utilbase/
    ${TASK_ROOT_DIR}/bbt_base_include/math/
    ${TASK_ROOT_DIR}/bbt_base_include/urdf_config/
    ${TASK_ROOT_DIR}/bbt_func_include/
    ${TASK_ROOT_DIR}/bbt_func_include/kinematic/
)

target_link_libraries(taskframe_launcher
    taskframe
    confbase
    osbase
    utilbase
    urdf_config
    pthread
    rt
    yaml-cpp
)

# === Unit tests ===
# 测试二进制输出到 output/（不污染部署目录 app/），运行目录同为 output/，
# 使 worker_manager_test 中的 "./launcher_test_worker.elf" 可解析。
find_package(GTest REQUIRED)
enable_testing()

set(LAUNCHER_TEST_OUTPUT ${PROJECT_SOURCE_DIR}/output)

add_executable(launcher_test_worker.elf
    unittest/launcher_test_worker.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/ipc_protocol.cpp
)
target_include_directories(launcher_test_worker.elf PRIVATE
    ${TASK_ROOT_DIR}/bbt_task_include/taskframe/
)
set_target_properties(launcher_test_worker.elf PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${LAUNCHER_TEST_OUTPUT})

add_executable(launcher_test_silent_worker.elf
    unittest/launcher_test_silent_worker.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/ipc_protocol.cpp
)
target_include_directories(launcher_test_silent_worker.elf PRIVATE
    ${TASK_ROOT_DIR}/bbt_task_include/taskframe/
)
set_target_properties(launcher_test_silent_worker.elf PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${LAUNCHER_TEST_OUTPUT})

add_executable(launcher_unittest
    unittest/config_parser_test.cpp
    unittest/launcher_shm_test.cpp
    unittest/worker_manager_test.cpp
    src/launcher.cpp
    src/worker_manager.cpp
    src/config_parser.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/ipc_protocol.cpp
)
target_include_directories(launcher_unittest PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TASK_ROOT_DIR}/bbt_task_include/taskframe/
    ${TASK_ROOT_DIR}/bbt_task_include/worker_base/
    ${TASK_ROOT_DIR}/bbt_task_include/
    ${TASK_ROOT_DIR}/bbt_base_include/
    ${TASK_ROOT_DIR}/bbt_base_include/osbase/
    ${TASK_ROOT_DIR}/bbt_base_include/confbase/
    ${TASK_ROOT_DIR}/bbt_base_include/utilbase/
    ${TASK_ROOT_DIR}/bbt_base_include/math/
    ${TASK_ROOT_DIR}/bbt_base_include/urdf_config/
    ${TASK_ROOT_DIR}/bbt_func_include/
    ${TASK_ROOT_DIR}/bbt_func_include/kinematic/
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(launcher_unittest
    ${GTEST_BOTH_LIBRARIES}
    taskframe
    confbase
    osbase
    utilbase
    urdf_config
    pthread
    rt
    yaml-cpp
)
set_target_properties(launcher_unittest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${LAUNCHER_TEST_OUTPUT})

add_test(NAME launcher_unittest COMMAND launcher_unittest)
set_tests_properties(launcher_unittest PROPERTIES
    WORKING_DIRECTORY ${LAUNCHER_TEST_OUTPUT})
