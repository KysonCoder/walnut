cmake_minimum_required(VERSION 3.10)

project(taskframe)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

set(CPU_ARCH "x86_64")
IF((CMAKE_CXX_COMPILER MATCHES  "[.*]*(aarch64)[.*]*") OR ((CMAKE_C_COMPILER MATCHES  "[.*]*(aarch64)[.*]*")))
    set(CPU_ARCH "aarch64")
ENDIF()
IF((CMAKE_CXX_COMPILER MATCHES  "[.*]*(arm)[.*]*") OR ((CMAKE_C_COMPILER MATCHES  "[.*]*(arm)[.*]*")))
    set(CPU_ARCH "aarch64")
ENDIF()

include(CMakeDependentOption)
include(GNUInstallDirs)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output/)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/output/)
SET(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/output/)

# Own source and headers
include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)

get_filename_component(TASK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../.. ABSOLUTE)

# External BBeamTech base libraries
include_directories(${TASK_ROOT_DIR}/bbt_base_include/)
include_directories(${TASK_ROOT_DIR}/bbt_base_include/osbase/)
include_directories(${TASK_ROOT_DIR}/bbt_base_include/confbase/)
include_directories(${TASK_ROOT_DIR}/bbt_base_include/utilbase/)
include_directories(${TASK_ROOT_DIR}/bbt_base_include/math/)
include_directories(${TASK_ROOT_DIR}/bbt_base_include/urdf_config/)
include_directories(${TASK_ROOT_DIR}/bbt_func_include/)
include_directories(${TASK_ROOT_DIR}/bbt_func_include/kinematic/)
include_directories(${TASK_ROOT_DIR}/bbt_base_include/drivechain/)

link_directories(${TASK_ROOT_DIR}/bbt_base_lib/)

add_compile_options(-D_GNU_SOURCE -Wformat-extra-args -Wno-format-security -Wno-format-extra-args -Wno-unused-variable)

set(ROBOT_TYPE "HUMANOID" CACHE STRING "Robot type: HUMANOID or DUAL_ARM")
set_property(CACHE ROBOT_TYPE PROPERTY STRINGS "HUMANOID" "DUAL_ARM")
message(STATUS "TaskFrame ROBOT_TYPE=${ROBOT_TYPE}")

add_library(${PROJECT_NAME}
    src/status_manager.cpp
    src/task_frame.cpp
    src/shm_base.cpp
    src/ipc_protocol.cpp
    src/tf_shm_region.cpp
    src/tf_event.cpp
    src/tf_msg_queue.cpp
    src/tf_feedback.cpp
    src/tf_fsm_adapter.cpp
    src/config_path_resolver.cpp
)

target_compile_definitions(${PROJECT_NAME} PUBLIC ROBOT_TYPE=${ROBOT_TYPE})

target_link_libraries(${PROJECT_NAME}
    confbase
    drivechain
    math
    osbase
    utilbase
    urdf_config
    rt
)

link_directories(${PROJECT_SOURCE_DIR}/output/)

include(CTest)
enable_testing()

# === PRE_BUILD: sync own headers to centralized bbt_task_include ===
add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        ${TASK_ROOT_DIR}/bbt_task_include/${PROJECT_NAME}/
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PROJECT_SOURCE_DIR}/include/
        ${TASK_ROOT_DIR}/bbt_task_include/${PROJECT_NAME}/
)

add_custom_command(TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND echo  Remove old ${PROJECT_NAME} sdk files
        COMMAND rm -Rf  ${PROJECT_SOURCE_DIR}/output/sdk/${CPU_ARCH}/bbt_include/${PROJECT_NAME}/*  -v
        COMMAND rm -Rf  ${PROJECT_SOURCE_DIR}/output/sdk/${CPU_ARCH}/bbt_lib/${PROJECT_NAME}/lib${PROJECT_NAME}.a  -v
        COMMAND echo Sync lib to centralized bbt_task_lib
        COMMAND cp -f ${PROJECT_SOURCE_DIR}/output/lib${PROJECT_NAME}.a ${TASK_ROOT_DIR}/bbt_task_lib/lib${PROJECT_NAME}.a -v
        COMMAND echo Copy to ${PROJECT_NAME} sdk
        COMMAND rm -Rf ${PROJECT_SOURCE_DIR}/output/${PROJECT_NAME}sdk
        COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/output/${PROJECT_NAME}sdk/${CPU_ARCH}/bbt_include/${PROJECT_NAME}
        COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/output/${PROJECT_NAME}sdk/${CPU_ARCH}/bbt_lib

        COMMAND cp -Rf ${PROJECT_SOURCE_DIR}/include/*.h*  ${PROJECT_SOURCE_DIR}/output/${PROJECT_NAME}sdk/${CPU_ARCH}/bbt_include/${PROJECT_NAME}/  -v

        COMMAND cp -f ${PROJECT_SOURCE_DIR}/output/lib${PROJECT_NAME}.a  ${PROJECT_SOURCE_DIR}/output/${PROJECT_NAME}sdk/${CPU_ARCH}/bbt_lib/  -v

        COMMAND echo Copy ${PROJECT_NAME} sdk into global sdks
        COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/output/sdk/${CPU_ARCH}/bbt_include/${PROJECT_NAME}
        COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/output/sdk/${CPU_ARCH}/bbt_lib
        COMMAND cp -Rf ${PROJECT_SOURCE_DIR}/output/${PROJECT_NAME}sdk/*  ${PROJECT_SOURCE_DIR}/output/sdk/    -v
        )

add_subdirectory(${PROJECT_SOURCE_DIR}/unittest)
add_subdirectory(${PROJECT_SOURCE_DIR}/worker_template)
