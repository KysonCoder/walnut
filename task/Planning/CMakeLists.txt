cmake_minimum_required(VERSION 3.10)

project(planning_task)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output/)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/output/)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/output/)

get_filename_component(TASK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../.. ABSOLUTE)

# Own headers
include_directories(${PROJECT_SOURCE_DIR}/include/)

# TaskFrame headers and libraries (published to bbt_task_*)
include_directories(${TASK_ROOT_DIR}/bbt_task_include/taskframe/)
include_directories(${TASK_ROOT_DIR}/bbt_task_include/worker_base/)
include_directories(${TASK_ROOT_DIR}/bbt_task_include/)
link_directories(${TASK_ROOT_DIR}/bbt_task_lib/)

# BBeamTech base libraries
include_directories(${TASK_ROOT_DIR}/bbt_base_include/)
include_directories(${TASK_ROOT_DIR}/bbt_base_include/osbase/)
include_directories(${TASK_ROOT_DIR}/bbt_base_include/confbase/)
include_directories(${TASK_ROOT_DIR}/bbt_base_include/utilbase/)
include_directories(${TASK_ROOT_DIR}/bbt_base_include/math/)
include_directories(${TASK_ROOT_DIR}/bbt_base_include/urdf_config/)
include_directories(${TASK_ROOT_DIR}/bbt_func_include/)
include_directories(${TASK_ROOT_DIR}/bbt_func_include/kinematic/)
# func 各库头文件按原名扁平互引（kinplanning README 约定），需各自目录在包含路径上
include_directories(${TASK_ROOT_DIR}/bbt_func_include/ppwodyn/)
include_directories(${TASK_ROOT_DIR}/bbt_func_include/kinplanning/)
include_directories(${TASK_ROOT_DIR}/bbt_func_include/action/)
include_directories(${TASK_ROOT_DIR}/bbt_func_include/spline/)
link_directories(${TASK_ROOT_DIR}/bbt_base_lib/)
# func 双 flavor 交付（2026-08-24 起）：HUMANOID 版在 humanoid/ 子目录；
# flavor 中立的 libspline/libservo/libfbbase 仍在根目录
link_directories(${TASK_ROOT_DIR}/bbt_func_lib/humanoid/)
link_directories(${TASK_ROOT_DIR}/bbt_func_lib/)

# === Planning task executable ===
add_executable(planning_task.elf
    src/planning_task_main.cpp
    src/planning_task.cpp
    src/planning_runner.cpp
)
target_link_libraries(planning_task.elf
    worker_base
    taskframe
    ppwodyn
    kinplanning
    action
    kinematic
    spline
    confbase
    drivechain
    urdf_config
    math
    osbase
    utilbase
    pthread
    rt
)

# Copy the worker ELF to the centralized app/ directory after building.
add_custom_command(TARGET planning_task.elf POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/../app
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:planning_task.elf> ${PROJECT_SOURCE_DIR}/../app/
    COMMENT "Copying planning_task.elf to app/"
)

# === Smoke test ===
add_executable(planning_task_smoke_test
    test/planning_task_smoke_test.cpp
    src/planning_task.cpp
    src/planning_runner.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/task_frame.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/status_manager.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/shm_base.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/tf_shm_region.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/tf_event.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/tf_msg_queue.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/tf_feedback.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/tf_fsm_adapter.cpp
    ${PROJECT_SOURCE_DIR}/../TaskFrame/src/config_path_resolver.cpp
)
target_link_libraries(planning_task_smoke_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ppwodyn
    kinplanning
    action
    kinematic
    spline
    confbase
    drivechain
    urdf_config
    math
    osbase
    utilbase
    rt
)
target_compile_options(planning_task_smoke_test PUBLIC -DTASK_FRAME_TEST -DPLANNING_TASK_TEST)
enable_testing()
get_filename_component(TASK_ROOT_DIR ${CMAKE_SOURCE_DIR}/.. ABSOLUTE)
add_test(NAME planning_task_smoke_test COMMAND planning_task_smoke_test)
set_tests_properties(planning_task_smoke_test PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..)

# === PlanningRunner unit test ===
add_executable(planning_runner_test
    test/planning_runner_test.cpp
    src/planning_runner.cpp
)
target_link_libraries(planning_runner_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ppwodyn
    kinplanning
    action
    kinematic
    spline
    confbase
    drivechain
    urdf_config
    math
    osbase
    utilbase
    rt
)
add_test(NAME planning_runner_test COMMAND planning_runner_test)
