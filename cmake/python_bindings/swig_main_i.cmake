# Defines provizio_dds_write_swig_main_i(), which builds the bundled SWIG
# interface file that combines every per-type IDL `.i` into a single Python
# module.
#
# Responsibilities:
#   1. Strip `%module ...` directives from each per-type `.i` file in place,
#      since the bundled file declares a single top-level %module.
#   2. Prepend `%ignore` directives for any constants that would otherwise
#      collide in the flattened SWIG-Python namespace. The detection happens
#      in detect_swig_dup_constants.py — see that file for background on why
#      duplicates appear with Fast-DDS-Gen v4 output.
#   3. Emit a top-level %module declaration and the list of `%include`s.
#
# Step 2 needs a real parser because semicolons inside C++ declarations
# collide with CMake's list separator if you try to do the scanning in pure
# CMake regex.

# Capture this file's directory at include time so the function can locate
# the sibling Python helper regardless of the caller's current list dir.
set(_PROVIZIO_DDS_SWIG_MAIN_I_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(provizio_dds_write_swig_main_i)
    set(_options "")
    set(_one_value_args OUTPUT MODULE_NAME IDLS_ROOT PYTHON3_EXECUTABLE)
    set(_multi_value_args IDL_FILES)
    cmake_parse_arguments(_ARG "${_options}" "${_one_value_args}" "${_multi_value_args}" ${ARGN})

    foreach(_required IN ITEMS OUTPUT MODULE_NAME IDLS_ROOT PYTHON3_EXECUTABLE)
        if(NOT _ARG_${_required})
            message(FATAL_ERROR "provizio_dds_write_swig_main_i: ${_required} is required")
        endif()
    endforeach()
    if(NOT _ARG_IDL_FILES)
        message(FATAL_ERROR "provizio_dds_write_swig_main_i: IDL_FILES is required and must not be empty")
    endif()

    list(SORT _ARG_IDL_FILES)

    set(_ignores_file "${CMAKE_CURRENT_BINARY_DIR}/swig_dup_constants_ignores.i")
    set(_detect_script "${_PROVIZIO_DDS_SWIG_MAIN_I_DIR}/detect_swig_dup_constants.py")
    execute_process(
        COMMAND "${_ARG_PYTHON3_EXECUTABLE}" "${_detect_script}"
            --idls-root "${_ARG_IDLS_ROOT}"
            --output "${_ignores_file}"
            ${_ARG_IDL_FILES}
        RESULT_VARIABLE _detect_rc
        OUTPUT_VARIABLE _detect_stdout
        ERROR_VARIABLE _detect_stderr
    )
    if(NOT _detect_rc EQUAL 0)
        message(FATAL_ERROR
            "detect_swig_dup_constants.py failed (exit ${_detect_rc}):\n"
            "stdout: ${_detect_stdout}\n"
            "stderr: ${_detect_stderr}")
    endif()

    file(WRITE "${_ARG_OUTPUT}" "%module ${_ARG_MODULE_NAME}\n\n")
    if(EXISTS "${_ignores_file}")
        file(READ "${_ignores_file}" _ignores_content)
        if(_ignores_content)
            file(APPEND "${_ARG_OUTPUT}" "${_ignores_content}\n")
        endif()
    endif()

    foreach(_idl IN LISTS _ARG_IDL_FILES)
        file(READ "${_idl}" _idl_content)
        string(REPLACE "%module" "//" _idl_content "${_idl_content}")
        file(WRITE "${_idl}" "${_idl_content}")
        # Quote the path so SWIG treats it as a single token even when the
        # build / source tree path contains spaces (common on Windows under
        # C:\Users\<Name>\...). Without quotes, an unquoted path with spaces
        # would be parsed as multiple arguments and %include would fail.
        file(APPEND "${_ARG_OUTPUT}" "%include \"${_idl}\"\n")
    endforeach()
endfunction()
