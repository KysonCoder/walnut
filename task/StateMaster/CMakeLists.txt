cmake_minimum_required(VERSION 3.10)
project(state_master)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# jsoncpp：开发包未安装时退化为运行时库完整路径
find_library(JSONCPP_LIBRARY NAMES jsoncpp)
if(NOT JSONCPP_LIBRARY)
    set(JSONCPP_LIBRARY /usr/lib/x86_64-linux-gnu/libjsoncpp.so.25)
endif()
include_directories(${GTEST_INCLUDE_DIRS})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output/)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/output/)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/output/)

get_filename_component(TASK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../.. ABSOLUTE)

# StateMaster own headers
include_directories(${PROJECT_SOURCE_DIR}/include/)

# HMI sub-module headers and vendored httplib
include_directories(${PROJECT_SOURCE_DIR}/hmi/)
include_directories(${PROJECT_SOURCE_DIR}/hmi/third_party/)

# Sibling task headers
include_directories(${PROJECT_SOURCE_DIR}/../Planning/include/)
include_directories(${PROJECT_SOURCE_DIR}/../Servo/include/)
include_directories(${PROJECT_SOURCE_DIR}/../Fieldbus/include/)

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
# Planning 集成 ppwodyn 后，集成测试重编 planning 源需要 func 各库扁平互引目录
include_directories(${TASK_ROOT_DIR}/bbt_func_include/ppwodyn/)
include_directories(${TASK_ROOT_DIR}/bbt_func_include/kinplanning/)
include_directories(${TASK_ROOT_DIR}/bbt_func_include/action/)
include_directories(${TASK_ROOT_DIR}/bbt_func_include/spline/)
link_directories(${TASK_ROOT_DIR}/bbt_base_lib/)
# HUMANOID flavor 优先；flavor 中立库（spline/servo/fbbase）在根目录
link_directories(${TASK_ROOT_DIR}/bbt_func_lib/humanoid/)
link_directories(${TASK_ROOT_DIR}/bbt_func_lib/)

# === StateMaster executable ===
add_executable(state_master.elf
    src/state_master_main.cpp
    src/state_master.cpp
    hmi/hmi_server.cpp
    hmi/hmi_interface.cpp
)
target_link_libraries(state_master.elf
    worker_base
    taskframe
    ${JSONCPP_LIBRARY}
    confbase
    drivechain
    osbase
    utilbase
    urdf_config
    pthread
    rt
)

# Copy the worker ELF to the centralized app/ directory after building.
add_custom_command(TARGET state_master.elf POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/../app
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:state_master.elf> ${PROJECT_SOURCE_DIR}/../app/
    COMMENT "Copying state_master.elf to app/"
)

# === Unit tests ===
add_executable(state_master_test
    test/state_master_test.cpp
    src/state_master.cpp
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
target_link_libraries(state_master_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    confbase
    drivechain
    osbase
    utilbase
    urdf_config
    rt
)
target_compile_options(state_master_test PUBLIC -DTASK_FRAME_TEST)

# === HMI interface unit test ===
add_executable(hmi_interface_test
    test/hmi_interface_test.cpp
    hmi/hmi_interface.cpp
    src/state_master.cpp
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
target_link_libraries(hmi_interface_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${JSONCPP_LIBRARY}
    confbase
    drivechain
    osbase
    utilbase
    urdf_config
    rt
)
target_compile_options(hmi_interface_test PUBLIC -DTASK_FRAME_TEST)
add_test(NAME hmi_interface_test COMMAND hmi_interface_test)

# === HMI server HTTP test ===
add_executable(hmi_server_test
    test/hmi_server_test.cpp
    hmi/hmi_server.cpp
    hmi/hmi_interface.cpp
    src/state_master.cpp
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
target_link_libraries(hmi_server_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${JSONCPP_LIBRARY}
    confbase
    drivechain
    osbase
    utilbase
    urdf_config
    rt
)
target_compile_options(hmi_server_test PUBLIC -DTASK_FRAME_TEST)
add_test(NAME hmi_server_test COMMAND hmi_server_test)

# === Multi-process integration test ===
add_executable(multi_process_integration_test
    test/multi_process_integration_test.cpp
    src/state_master.cpp
    ${CMAKE_SOURCE_DIR}/../Planning/src/planning_task.cpp
    ${CMAKE_SOURCE_DIR}/../Planning/src/planning_runner.cpp
    ${CMAKE_SOURCE_DIR}/../Servo/src/servo_task.cpp
    ${CMAKE_SOURCE_DIR}/../Servo/src/servo_engine.cpp
    ${CMAKE_SOURCE_DIR}/../Fieldbus/src/fieldbus_task.cpp
    ${CMAKE_SOURCE_DIR}/../Fieldbus/src/servo_ref.cpp
    ${CMAKE_SOURCE_DIR}/../Fieldbus/src/motion_stop.cpp
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
target_link_libraries(multi_process_integration_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ppwodyn
    kinplanning
    action
    kinematic
    servo
    spline
    confbase
    drivechain
    osbase
    utilbase
    urdf_config
    math
    rt
)
target_compile_options(multi_process_integration_test PUBLIC
    -DSTATE_MASTER_TEST
    -DPLANNING_TASK_TEST
    -DSERVO_TASK_TEST
    -DFIELDBUS_TASK_TEST
    -DTASK_FRAME_TEST
)
enable_testing()
get_filename_component(TASK_ROOT_DIR ${CMAKE_SOURCE_DIR}/.. ABSOLUTE)
add_test(NAME state_master_test COMMAND state_master_test)
set_tests_properties(state_master_test PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..)
add_test(NAME multi_process_integration_test COMMAND multi_process_integration_test)
set_tests_properties(multi_process_integration_test PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..)

# === Full-app state machine integration test ===
# 需要真实 EtherCAT 从站在线 + 启动完整 task/app 应用，不注册 ctest，手动运行：
#   sudo ./output/full_app_state_machine_test
add_executable(full_app_state_machine_test
    test/full_app_state_machine_test.cpp
)
target_link_libraries(full_app_state_machine_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    rt
)

# === Web FSM manual tester (tool, not a ctest test) ===
add_executable(web_fsm_tester
    tools/web_fsm_tester/main.cpp
    tools/web_fsm_tester/shm_watcher.cpp
    tools/web_fsm_tester/hmi_client.cpp
    tools/web_fsm_tester/app_control.cpp
    tools/web_fsm_tester/api_utils.cpp
    tools/web_fsm_tester/sequence_runner.cpp
)
target_link_libraries(web_fsm_tester
    ${CMAKE_THREAD_LIBS_INIT}
    utilbase
    ${JSONCPP_LIBRARY}
    rt
)

add_executable(web_fsm_tester_test
    tools/web_fsm_tester/web_fsm_tester_test.cpp
    tools/web_fsm_tester/api_utils.cpp
    tools/web_fsm_tester/shm_watcher.cpp
    tools/web_fsm_tester/sequence_runner.cpp
    tools/web_fsm_tester/hmi_client.cpp
)
target_link_libraries(web_fsm_tester_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    utilbase
    ${JSONCPP_LIBRARY}
    rt
)
add_test(NAME web_fsm_tester_test COMMAND web_fsm_tester_test)
