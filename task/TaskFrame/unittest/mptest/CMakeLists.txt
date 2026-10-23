cmake_minimum_required(VERSION 3.10)

enable_testing()

get_filename_component(TASK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../.. ABSOLUTE)

# External BBeamTech base libraries
include_directories(${TASK_ROOT_DIR}/bbt_base_include/)

# Own headers
include_directories(${PROJECT_SOURCE_DIR}/../include/)

# taskframe library output and base libraries
link_directories(${TASK_ROOT_DIR}/bbt_task_lib/)
link_directories(${TASK_ROOT_DIR}/bbt_base_lib/)

link_libraries(taskframe)
link_libraries(rt)

# === Multi-process fork test: tf_event ===
add_executable(tf_event_fork_test tf_event_fork_test.cpp)
target_link_libraries(tf_event_fork_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME tf_event_fork_test COMMAND tf_event_fork_test)

# === Multi-process fork test: tf_msg_queue ===
add_executable(tf_msg_queue_fork_test tf_msg_queue_fork_test.cpp)
target_link_libraries(tf_msg_queue_fork_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
add_test(NAME tf_msg_queue_fork_test COMMAND tf_msg_queue_fork_test)

# === Multi-process fork test: tf_feedback ===
add_executable(tf_feedback_fork_test tf_feedback_fork_test.cpp)
target_link_libraries(tf_feedback_fork_test
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    taskframe
    rt
)
target_compile_definitions(tf_feedback_fork_test PUBLIC ROBOT_TYPE=${ROBOT_TYPE})
add_test(NAME tf_feedback_fork_test COMMAND tf_feedback_fork_test)

# === Test worker executables (used by integration tests) ===
add_executable(test_worker.elf test_worker_main.cpp)
target_link_libraries(test_worker.elf
    worker_base
    taskframe
    pthread
    rt
)

add_executable(test_state_master_worker.elf test_state_master_worker.cpp)
target_link_libraries(test_state_master_worker.elf worker_base taskframe pthread rt)

add_executable(test_planning_worker.elf test_planning_worker.cpp)
target_link_libraries(test_planning_worker.elf worker_base taskframe pthread rt)

add_executable(test_servo_worker.elf test_servo_worker.cpp)
target_link_libraries(test_servo_worker.elf worker_base taskframe pthread rt)

add_executable(test_fieldbus_worker.elf test_fieldbus_worker.cpp)
target_link_libraries(test_fieldbus_worker.elf worker_base taskframe pthread rt)

get_filename_component(TASK_ROOT_DIR ${CMAKE_SOURCE_DIR}/.. ABSOLUTE)

# === Integration test: launcher + worker ===
add_executable(test_launcher_worker test_launcher_worker.cpp)
target_link_libraries(test_launcher_worker
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    rt
    pthread
)
add_test(NAME LauncherWorkerTest.LauncherStartsWorker COMMAND test_launcher_worker --gtest_filter=LauncherWorkerTest.LauncherStartsWorker)
add_test(NAME LauncherWorkerTest.LauncherConfigParse COMMAND test_launcher_worker --gtest_filter=LauncherWorkerTest.LauncherConfigParse)
add_test(NAME LauncherWorkerTest.LauncherBinaryExists COMMAND test_launcher_worker --gtest_filter=LauncherWorkerTest.LauncherBinaryExists)
add_test(NAME LauncherWorkerTest.WorkerBinaryExists COMMAND test_launcher_worker --gtest_filter=LauncherWorkerTest.WorkerBinaryExists)
set_tests_properties(
    LauncherWorkerTest.LauncherStartsWorker
    LauncherWorkerTest.LauncherConfigParse
    LauncherWorkerTest.LauncherBinaryExists
    LauncherWorkerTest.WorkerBinaryExists
    PROPERTIES WORKING_DIRECTORY ${TASK_ROOT_DIR}
)

# === Four-process integration test ===
add_executable(test_four_process test_four_process.cpp)
target_link_libraries(test_four_process
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    rt
    pthread
)
add_test(NAME FourProcessTest.LauncherStartsAllFourWorkers COMMAND test_four_process --gtest_filter=FourProcessTest.LauncherStartsAllFourWorkers)
add_test(NAME FourProcessTest.ConfigFileExists COMMAND test_four_process --gtest_filter=FourProcessTest.ConfigFileExists)
add_test(NAME FourProcessTest.AllWorkerBinariesExist COMMAND test_four_process --gtest_filter=FourProcessTest.AllWorkerBinariesExist)
set_tests_properties(
    FourProcessTest.LauncherStartsAllFourWorkers
    FourProcessTest.ConfigFileExists
    FourProcessTest.AllWorkerBinariesExist
    PROPERTIES WORKING_DIRECTORY ${TASK_ROOT_DIR}
)
