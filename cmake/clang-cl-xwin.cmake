# Cross-compile x86_64 Windows binaries on Linux using LLVM 20 and an xwin SDK.
# Pass --toolchain cmake/clang-cl-xwin.cmake; override LLVM_BIN, XWIN_DIR or WINE as needed.
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

set(LLVM_BIN /usr/lib/llvm-20/bin CACHE PATH "LLVM tool directory")
set(XWIN_DIR /opt/xwin CACHE PATH "xwin CRT and Windows SDK directory")
set(WINE /usr/lib/wine/wine64 CACHE FILEPATH "64-bit Wine loader for CTest")
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES LLVM_BIN XWIN_DIR WINE)

set(CMAKE_C_COMPILER "${LLVM_BIN}/clang-cl")
set(CMAKE_CXX_COMPILER "${LLVM_BIN}/clang-cl")
set(CMAKE_C_COMPILER_TARGET x86_64-pc-windows-msvc)
set(CMAKE_CXX_COMPILER_TARGET x86_64-pc-windows-msvc)
set(CMAKE_LINKER "${LLVM_BIN}/lld-link")
set(CMAKE_RC_COMPILER "${LLVM_BIN}/llvm-rc")
set(CMAKE_MT "${LLVM_BIN}/llvm-mt")
set(CMAKE_AR "${LLVM_BIN}/llvm-lib")
set(CMAKE_CROSSCOMPILING_EMULATOR "${WINE}" CACHE STRING "Windows executable emulator")

set(xwin_include_flags "")
foreach(include_dir IN ITEMS crt/include sdk/include/ucrt sdk/include/um sdk/include/shared sdk/include/winrt)
    string(APPEND xwin_include_flags " -imsvc\"${XWIN_DIR}/${include_dir}\"")
endforeach()
set(CMAKE_C_FLAGS_INIT "${xwin_include_flags}")
set(CMAKE_CXX_FLAGS_INIT "${xwin_include_flags}")

set(xwin_link_flags "")
foreach(lib_dir IN ITEMS crt/lib/x86_64 sdk/lib/um/x86_64 sdk/lib/ucrt/x86_64)
    string(APPEND xwin_link_flags " /libpath:\"${XWIN_DIR}/${lib_dir}\"")
endforeach()
foreach(target_type IN ITEMS EXE SHARED MODULE)
    set(CMAKE_${target_type}_LINKER_FLAGS_INIT "${xwin_link_flags}")
endforeach()
