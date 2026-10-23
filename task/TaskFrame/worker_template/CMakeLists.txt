get_filename_component(TASK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../.. ABSOLUTE)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/output/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/output/)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/output/)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/output/)

add_library(worker_base STATIC
    src/worker_base.cpp
)

target_include_directories(worker_base PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(worker_base PUBLIC
    taskframe
    pthread
    rt
)

# Publish worker_base headers and library to centralized bbt_task_* directories
add_custom_command(TARGET worker_base POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        ${TASK_ROOT_DIR}/bbt_task_include/worker_base/
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/include/
        ${TASK_ROOT_DIR}/bbt_task_include/worker_base/
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/output/libworker_base.a
        ${TASK_ROOT_DIR}/bbt_task_lib/libworker_base.a
    COMMENT "Publishing worker_base SDK to bbt_task_include / bbt_task_lib"
)
