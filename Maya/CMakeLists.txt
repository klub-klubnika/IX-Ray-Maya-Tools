option(IXRAY_DOWNLOAD_MAYA_SDKS "Download Maya SDKs during CMake configuration" ON)

function(setup_maya_sdk version)
    set(IXR_MAYA_SDK_${version}_ZIP ${DEP_DIR}/Autodesk.Maya.Sdk.${version}.zip)
    set(IXR_MAYA_SDK_${version}_URL https://github.com/ixray-team/ixray-packages/releases/download/d2024.5.3/Autodesk.Maya.Sdk.${version}.zip)
    set(IXR_MAYA_SDK_${version}_DIR ${DEP_DIR}/maya_sdk_${version})
    if (IXRAY_DOWNLOAD_MAYA_SDKS)
        download_and_extract_sdk(${IXR_MAYA_SDK_${version}_URL} ${IXR_MAYA_SDK_${version}_ZIP} ${IXR_MAYA_SDK_${version}_DIR})
    endif()
    set(IXR_MAYA_SDK_${version} ${IXR_MAYA_SDK_${version}_DIR}/ PARENT_SCOPE)
endfunction()

foreach(version 2024 2025 2026 2027)
    setup_maya_sdk(${version})
endforeach()

project(MayaTools)

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/xray_bone_collision.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/xray_motion_browser.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/xray_re_object_options.mel")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/xray_bone_collision.py" MAYA_BONE_COLLISION_SCRIPT)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/xray_motion_browser.py" MAYA_MOTION_BROWSER_SCRIPT)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/xray_re_object_options.mel" MAYA_OBJECT_OPTIONS_SCRIPT)
string(REPLACE "\n" "\n)IXR_PY\"\nR\"IXR_PY(" MAYA_BONE_COLLISION_SCRIPT "${MAYA_BONE_COLLISION_SCRIPT}")
string(REPLACE "\n" "\n)IXR_PY\"\nR\"IXR_PY(" MAYA_MOTION_BROWSER_SCRIPT "${MAYA_MOTION_BROWSER_SCRIPT}")
string(REPLACE "\n" "\n)IXR_MEL\"\nR\"IXR_MEL(" MAYA_OBJECT_OPTIONS_SCRIPT "${MAYA_OBJECT_OPTIONS_SCRIPT}")
configure_file(maya_bone_collision_script.h.in maya_bone_collision_script.h @ONLY)
configure_file(maya_motion_browser_script.h.in maya_motion_browser_script.h @ONLY)
configure_file(maya_options_script.h.in maya_options_script.h @ONLY)

file(GLOB_RECURSE MAYA_TOOLS_ALL_FILES CONFIGURE_DEPENDS "*.cpp" "*.cxx" "*.h")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${MAYA_TOOLS_ALL_FILES})

function(add_maya_tools_target TARGET_NAME MAYA_VERSION MAYA_SDK_DIR)
    add_library(${TARGET_NAME} SHARED ${MAYA_TOOLS_ALL_FILES})

    target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}" "${MAYA_SDK_DIR}")
    target_link_directories(${TARGET_NAME} PRIVATE "${MAYA_SDK_DIR}/lib")

    target_link_libraries(${TARGET_NAME} PUBLIC
        PCore
        OpenMaya
        OpenMayaAnim
        Foundation
    )

    set_target_properties(${TARGET_NAME} PROPERTIES
        DISABLE_PRECOMPILE_HEADERS ON
        OUTPUT_NAME "ixray_maya_tools_${MAYA_VERSION}"
        SUFFIX ".mll"
    )

    target_compile_definitions(${TARGET_NAME} PRIVATE IXR_MAYA_VERSION=${MAYA_VERSION})
endfunction()

add_maya_tools_target(MayaTools_2024 2024 "${IXR_MAYA_SDK_2024}")
add_maya_tools_target(MayaTools_2025 2025 "${IXR_MAYA_SDK_2025}")
add_maya_tools_target(MayaTools_2026 2026 "${IXR_MAYA_SDK_2026}")
add_maya_tools_target(MayaTools_2027 2027 "${IXR_MAYA_SDK_2027}")

option(IXR_BUILD_MAYA_OGF_TESTS "Build skeletal OGF compiler regression tests" OFF)
if(IXR_BUILD_MAYA_OGF_TESTS)
    add_executable(MayaOgfExportTests ../tests/ogf_export.cxx ogf_export.cxx)
    target_include_directories(MayaOgfExportTests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(MayaOgfExportTests PRIVATE PCore)
    set_target_properties(MayaOgfExportTests PROPERTIES DISABLE_PRECOMPILE_HEADERS ON)
endif()
