cmake_minimum_required(VERSION 3.26)

project(IXRayMayaTools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(DEP_DIR "${CMAKE_BINARY_DIR}/deps" CACHE PATH "Directory for downloaded Maya SDKs")

function(download_and_extract_sdk url archive destination)
    if (NOT EXISTS "${destination}")
        file(MAKE_DIRECTORY "${DEP_DIR}")
        if (NOT EXISTS "${archive}")
            file(DOWNLOAD "${url}" "${archive}" SHOW_PROGRESS STATUS status)
            list(GET status 0 status_code)
            if (NOT status_code EQUAL 0)
                list(GET status 1 status_message)
                message(FATAL_ERROR "Unable to download ${url}: ${status_message}")
            endif()
        endif()
        file(MAKE_DIRECTORY "${destination}")
        file(ARCHIVE_EXTRACT INPUT "${archive}" DESTINATION "${destination}")
    endif()
endfunction()

add_subdirectory(PCore)
add_subdirectory(Maya)

set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT MayaTools_2024)
