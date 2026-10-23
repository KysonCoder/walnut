# verify_flavor.cmake — 消费侧 func 库 flavor 校验（spec 2026-09-27 §3.2）。
# task 侧用法：
#   include(<path>/bbt_func_lib/cmake/verify_flavor.cmake)
#   verify_func_flavor(<path>/bbt_func_lib/libkinematic_humanoid.a humanoid)
# 文件名后缀与库内身份符号不符时 configure 期 FATAL_ERROR。
# 仅适用于带后缀的双 flavor 库；flavor 无关库（spline/servo/action_fsm 等）
# 无身份符号，不要用本函数校验。

function(verify_func_flavor LIB_FILE EXPECTED_FLAVOR)
  if(NOT EXISTS "${LIB_FILE}")
    message(FATAL_ERROR "verify_func_flavor: not a file: ${LIB_FILE}")
  endif()
  execute_process(
    COMMAND nm "${LIB_FILE}"
    OUTPUT_VARIABLE _nm_out
    ERROR_VARIABLE _nm_err
    RESULT_VARIABLE _nm_rc
  )
  if(NOT _nm_rc EQUAL 0)
    message(FATAL_ERROR "verify_func_flavor: nm failed on ${LIB_FILE}: ${_nm_err}")
  endif()
  string(FIND "${_nm_out}" "g_func_lib_flavor_${EXPECTED_FLAVOR}" _hit)
  if(_hit EQUAL -1)
    message(FATAL_ERROR
      "verify_func_flavor FAILED: ${LIB_FILE} is NOT flavor '${EXPECTED_FLAVOR}' "
      "identity symbol g_func_lib_flavor_${EXPECTED_FLAVOR} missing. "
      "Check the library file name suffix.")
  endif()
  message(STATUS "verify_func_flavor OK: ${LIB_FILE} is ${EXPECTED_FLAVOR}")
endfunction()
