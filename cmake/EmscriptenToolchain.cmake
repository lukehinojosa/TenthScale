# Finds emsdk and loads its toolchain. It also reads the saved EMSDK setting directly, so an IDE or
# terminal opened before install.bat doesn't need restarting.
set(emsdkCandidates "$ENV{EMSDK}")
if(CMAKE_HOST_WIN32)
    cmake_host_system_information(RESULT savedEmsdk QUERY WINDOWS_REGISTRY "HKCU/Environment" VALUE "EMSDK" ERROR_VARIABLE registryError)
    list(APPEND emsdkCandidates "${savedEmsdk}" "$ENV{USERPROFILE}/tools/emsdk")
else()
    list(APPEND emsdkCandidates "$ENV{HOME}/tools/emsdk")
endif()

set(emsdkRoot "")
foreach(candidate IN LISTS emsdkCandidates)
    file(TO_CMAKE_PATH "${candidate}" candidate)
    if(candidate AND EXISTS "${candidate}/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")
        set(emsdkRoot "${candidate}")
        break()
    endif()
endforeach()

if(NOT emsdkRoot)
    message(FATAL_ERROR "emsdk was not found. Run install.bat (Windows) or set EMSDK to the emsdk folder.")
endif()

include("${emsdkRoot}/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")
