cmake_minimum_required(VERSION 3.10)

project(mirrorme_robot_sdk_examples LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/utf-8)
endif()

find_package(Threads REQUIRED)

add_library(mirrorme_robot_sdk SHARED IMPORTED)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" MIRRORME_ROBOT_SDK_ARCH)
if(MIRRORME_ROBOT_SDK_ARCH MATCHES "^(amd64|x64)$")
    set(MIRRORME_ROBOT_SDK_ARCH x86_64)
elseif(MIRRORME_ROBOT_SDK_ARCH STREQUAL "arm64")
    set(MIRRORME_ROBOT_SDK_ARCH aarch64)
endif()

if(WIN32)
    set(MIRRORME_ROBOT_SDK_RUNTIME_LIBRARY_CANDIDATES
        "${CMAKE_CURRENT_LIST_DIR}/bin/mirrorme_robot_sdk_${MIRRORME_ROBOT_SDK_ARCH}.dll"
        "${CMAKE_CURRENT_LIST_DIR}/lib/mirrorme_robot_sdk_${MIRRORME_ROBOT_SDK_ARCH}.dll"
    )
    set(MIRRORME_ROBOT_SDK_IMPORT_LIBRARY_CANDIDATES
        "${CMAKE_CURRENT_LIST_DIR}/lib/mirrorme_robot_sdk_${MIRRORME_ROBOT_SDK_ARCH}.lib"
        "${CMAKE_CURRENT_LIST_DIR}/bin/mirrorme_robot_sdk_${MIRRORME_ROBOT_SDK_ARCH}.lib"
    )
elseif(APPLE)
    set(MIRRORME_ROBOT_SDK_LIBRARY_CANDIDATES
        "${CMAKE_CURRENT_LIST_DIR}/lib/libmirrorme_robot_sdk_${MIRRORME_ROBOT_SDK_ARCH}.dylib"
    )
else()
    set(MIRRORME_ROBOT_SDK_LIBRARY_CANDIDATES
        "${CMAKE_CURRENT_LIST_DIR}/lib/libmirrorme_robot_sdk_${MIRRORME_ROBOT_SDK_ARCH}.so"
    )
endif()

foreach(MIRRORME_ROBOT_SDK_RUNTIME_LIBRARY_CANDIDATE IN LISTS MIRRORME_ROBOT_SDK_RUNTIME_LIBRARY_CANDIDATES)
    if(EXISTS "${MIRRORME_ROBOT_SDK_RUNTIME_LIBRARY_CANDIDATE}")
        set(MIRRORME_ROBOT_SDK_RUNTIME_LIBRARY_PATH "${MIRRORME_ROBOT_SDK_RUNTIME_LIBRARY_CANDIDATE}")
        break()
    endif()
endforeach()

foreach(MIRRORME_ROBOT_SDK_IMPORT_LIBRARY_CANDIDATE IN LISTS MIRRORME_ROBOT_SDK_IMPORT_LIBRARY_CANDIDATES MIRRORME_ROBOT_SDK_LIBRARY_CANDIDATES)
    if(EXISTS "${MIRRORME_ROBOT_SDK_IMPORT_LIBRARY_CANDIDATE}")
        set(MIRRORME_ROBOT_SDK_IMPORT_LIBRARY_PATH "${MIRRORME_ROBOT_SDK_IMPORT_LIBRARY_CANDIDATE}")
        break()
    endif()
endforeach()

set_target_properties(mirrorme_robot_sdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/include"
    INTERFACE_LINK_LIBRARIES Threads::Threads
)

if(WIN32)
    if(NOT MIRRORME_ROBOT_SDK_RUNTIME_LIBRARY_PATH)
        message(FATAL_ERROR "No MirrorMe Robot SDK runtime DLL found for architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
    if(NOT MIRRORME_ROBOT_SDK_IMPORT_LIBRARY_PATH)
        message(FATAL_ERROR "No MirrorMe Robot SDK import library found for architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
    set_target_properties(mirrorme_robot_sdk PROPERTIES
        IMPORTED_LOCATION "${MIRRORME_ROBOT_SDK_RUNTIME_LIBRARY_PATH}"
        IMPORTED_IMPLIB "${MIRRORME_ROBOT_SDK_IMPORT_LIBRARY_PATH}"
    )
else()
    if(NOT MIRRORME_ROBOT_SDK_IMPORT_LIBRARY_PATH)
        message(FATAL_ERROR "No MirrorMe Robot SDK shared library found for architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
    set_target_properties(mirrorme_robot_sdk PROPERTIES
        IMPORTED_LOCATION "${MIRRORME_ROBOT_SDK_IMPORT_LIBRARY_PATH}"
    )
endif()

function(add_mirrorme_robot_sdk_example target source)
    add_executable(${target}
        ${source}
    )
    target_link_libraries(${target}
        PRIVATE
            mirrorme_robot_sdk
    )
    set_target_properties(${target} PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_LIST_DIR}/lib"
    )
    if(WIN32)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${MIRRORME_ROBOT_SDK_RUNTIME_LIBRARY_PATH}"
                "$<TARGET_FILE_DIR:${target}>"
            VERBATIM
        )
    endif()
endfunction()

add_mirrorme_robot_sdk_example(request_robot_state_example example/request_robot_state_example.cpp)
add_mirrorme_robot_sdk_example(motion_level_control_example example/motion_level_control_example.cpp)
add_mirrorme_robot_sdk_example(pose_tracking_example example/pose_tracking_example.cpp)
add_mirrorme_robot_sdk_example(joint_level_control_example example/joint_level_control_example.cpp)
add_mirrorme_robot_sdk_example(motion_level_control_server_example example/motion_level_control_server_example.cpp)
add_mirrorme_robot_sdk_example(motion_level_control_client_example example/motion_level_control_client_example.cpp)
if(WIN32)
    target_link_libraries(motion_level_control_server_example PRIVATE ws2_32)
    target_link_libraries(motion_level_control_client_example PRIVATE ws2_32)
endif()

# Allow the installed SDK repository to add downstream-only targets without
# carrying their sources in this upstream package.
include(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/mirrorme_robot_sdk_extensions.cmake"
    OPTIONAL
)
