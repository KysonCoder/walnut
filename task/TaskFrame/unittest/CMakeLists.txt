cmake_minimum_required(VERSION 3.10)
project(taskframe_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../output/${PROJECT_NAME}/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../output/${PROJECT_NAME}/)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../output/${PROJECT_NAME}/)
SET(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/../output/${PROJECT_NAME}/)

set(ROBOT_TYPE "HUMANOID" CACHE STRING "Robot type: HUMANOID or DUAL_ARM")
set_property(CACHE ROBOT_TYPE PROPERTY STRINGS "HUMANOID" "DUAL_ARM")
message(STATUS "TaskFrame ROBOT_TYPE=${ROBOT_TYPE}")

enable_testing()

get_filename_component(TASK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../.. ABSOLUTE)

# External BBeamTech base libraries
include_directories(${TASK_ROOT_DIR}/bbt_base_include/)

# Own headers
include_directories(${PROJECT_SOURCE_DIR}/../include/)

# taskframe library output and base libraries
link_directories(${TASK_ROOT_DIR}/bbt_task_lib/)
link_directories(${TASK_ROOT_DIR}/bbt_base_lib/)

# === Unit test: status_manager ===
add_executable(status_manager_test status_manager_test.cpp)
target_link_libraries(status_manager_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME status_manager_test COMMAND status_manager_test)

# === Unit test: main_fsm ===
add_executable(main_fsm_test main_fsm_test.cpp)
target_link_libraries(main_fsm_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME main_fsm_test COMMAND main_fsm_test)

# === Unit test: shm_ptr ===
add_executable(test_shm_ptr test_shm_ptr.cpp)
target_link_libraries(test_shm_ptr
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME test_shm_ptr COMMAND test_shm_ptr)

# === Unit test: ipc_protocol ===
add_executable(test_ipc_protocol test_ipc_protocol.cpp)
target_link_libraries(test_ipc_protocol
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME test_ipc_protocol COMMAND test_ipc_protocol)

# === Unit test: task_frame (recompiles sources with -DTASK_FRAME_TEST) ===
add_executable(task_frame_test
    task_frame_test.cpp
    dummytask.cpp
    ../src/task_frame.cpp
    ../src/status_manager.cpp
    ../src/shm_base.cpp
    ../src/tf_shm_region.cpp
    ../src/tf_event.cpp
    ../src/tf_msg_queue.cpp
    ../src/tf_feedback.cpp
    ../src/tf_fsm_adapter.cpp
    ../src/config_path_resolver.cpp
)
target_link_libraries(task_frame_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    confbase
    drivechain
    math
    osbase
    utilbase
    urdf_config
    rt
)
target_compile_definitions(task_frame_test PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
target_compile_definitions(task_frame_test PUBLIC TASK_FRAME_TEST)
add_test(NAME task_frame_test COMMAND task_frame_test)
set_tests_properties(task_frame_test PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)

# === Unit test: task_frame_init robustness ===
add_executable(task_frame_init_test task_frame_init_test.cpp)
target_link_libraries(task_frame_init_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    confbase
    osbase
    utilbase
    urdf_config
    rt
)
target_compile_definitions(task_frame_init_test PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
target_compile_options(task_frame_init_test PUBLIC -DTASK_FRAME_TEST)
add_test(NAME task_frame_init_test COMMAND task_frame_init_test)
set_tests_properties(task_frame_init_test PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..)

# === Unit test: tf_shm_region ===
add_executable(tf_shm_region_test tf_shm_region_test.cpp)
target_link_libraries(tf_shm_region_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME tf_shm_region_test COMMAND tf_shm_region_test)

# === Unit test: tf_event ===
add_executable(tf_event_test tf_event_test.cpp)
target_link_libraries(tf_event_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME tf_event_test COMMAND tf_event_test)

# === Unit test: tf_msg_queue ===
add_executable(tf_msg_queue_test tf_msg_queue_test.cpp)
target_link_libraries(tf_msg_queue_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME tf_msg_queue_test COMMAND tf_msg_queue_test)

# === Unit test: tf_feedback ===
add_executable(tf_feedback_test tf_feedback_test.cpp)
target_link_libraries(tf_feedback_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME tf_feedback_test COMMAND tf_feedback_test)

# === Unit test: tf_fsm_adapter ===
add_executable(tf_fsm_adapter_test tf_fsm_adapter_test.cpp)
target_link_libraries(tf_fsm_adapter_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME tf_fsm_adapter_test COMMAND tf_fsm_adapter_test)

# === Unit test: robot_config_layout ===
add_executable(test_robot_config_layout test_robot_config_layout.cpp)
target_link_libraries(test_robot_config_layout
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME test_robot_config_layout COMMAND test_robot_config_layout)
set_tests_properties(test_robot_config_layout PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)

# === Unit test: robot_config_integration ===
add_executable(test_robot_config_integration test_robot_config_integration.cpp)
target_link_libraries(test_robot_config_integration
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    confbase
    drivechain
    osbase
    utilbase
    urdf_config
    rt
)
target_compile_definitions(test_robot_config_integration PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
add_test(NAME test_robot_config_integration COMMAND test_robot_config_integration)
set_tests_properties(test_robot_config_integration PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)

# === Unit test: robot_config_multiprocess ===
add_executable(test_robot_config_multiprocess test_robot_config_multiprocess.cpp)
target_link_libraries(test_robot_config_multiprocess
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    confbase
    drivechain
    osbase
    utilbase
    urdf_config
    rt
)
target_compile_definitions(test_robot_config_multiprocess PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
add_test(NAME test_robot_config_multiprocess COMMAND test_robot_config_multiprocess)
set_tests_properties(test_robot_config_multiprocess PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)

# === Unit test: dyngrp ===
add_executable(test_dyngrp test_dyngrp.cpp)
target_link_libraries(test_dyngrp
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    confbase
    drivechain
    osbase
    utilbase
    urdf_config
    rt
)
target_compile_definitions(test_dyngrp PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
add_test(NAME test_dyngrp COMMAND test_dyngrp)
set_tests_properties(test_dyngrp PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)

# === Unit test: joint_pos ===
add_executable(test_joint_pos test_joint_pos.cpp)
target_link_libraries(test_joint_pos
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    confbase
    drivechain
    osbase
    utilbase
    urdf_config
    rt
)
target_compile_definitions(test_joint_pos PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
add_test(NAME test_joint_pos COMMAND test_joint_pos)
set_tests_properties(test_joint_pos PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)

# === Unit test: robot_type ===
add_executable(test_robot_type test_robot_type.cpp)
target_link_libraries(test_robot_type
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    confbase
    rt
)
target_compile_definitions(test_robot_type PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
add_test(NAME test_robot_type COMMAND test_robot_type)

# === Unit test: robot_config_new_fields ===
add_executable(test_robot_config_new_fields test_robot_config_new_fields.cpp)
target_link_libraries(test_robot_config_new_fields
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    confbase
    osbase
    utilbase
    urdf_config
    drivechain
    rt
)
target_compile_definitions(test_robot_config_new_fields PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
add_test(NAME test_robot_config_new_fields COMMAND test_robot_config_new_fields)
set_tests_properties(test_robot_config_new_fields PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)

# === Unit test: robot_config_validation ===
add_executable(test_robot_config_validation test_robot_config_validation.cpp)
target_link_libraries(test_robot_config_validation
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    confbase
    osbase
    utilbase
    urdf_config
    drivechain
    rt
)
target_compile_definitions(test_robot_config_validation PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
add_test(NAME test_robot_config_validation COMMAND test_robot_config_validation)
set_tests_properties(test_robot_config_validation PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)

# === RT hot-path verification: 1 kHz x 5 s ===
add_executable(tf_hotpath_rt_test tf_hotpath_rt_test.cpp)
target_link_libraries(tf_hotpath_rt_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
target_compile_definitions(tf_hotpath_rt_test PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
add_test(NAME tf_hotpath_rt_test COMMAND tf_hotpath_rt_test)

# === Unit test: tf_types ===
add_executable(tf_types_test tf_types_test.cpp)
target_link_libraries(tf_types_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    confbase
    rt
)
target_compile_definitions(tf_types_test PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
add_test(NAME tf_types_test COMMAND tf_types_test)

add_subdirectory(${PROJECT_SOURCE_DIR}/mptest)
