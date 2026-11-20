# Cross-compile Windows binaries on Linux using LLVM 20 and an xwin CRT/SDK splat.
# Pass --toolchain cmake/clang-cl-xwin.cmake and -DXWIN_ARCH=x86_64 (default) or x86.
# Override LLVM_BIN or XWIN_DIR as needed (each arch has its own splat directory on the runner).
set(XWIN_ARCH x86_64 CACHE STRING "Target architecture: x86_64 or x86")
set_property(CACHE XWIN_ARCH PROPERTY STRINGS x86_64 x86)

set(CMAKE_SYSTEM_NAME Windows)
if(XWIN_ARCH STREQUAL "x86")
    set(CMAKE_SYSTEM_PROCESSOR x86)
    set(xwin_target i686-pc-windows-msvc)
    set(xwin_default_dir /opt/xwin-x86)
else()
    set(CMAKE_SYSTEM_PROCESSOR x86_64)
    set(xwin_target x86_64-pc-windows-msvc)
    set(xwin_default_dir /opt/xwin)
endif()

set(LLVM_BIN /usr/lib/llvm-20/bin CACHE PATH "LLVM tool directory")
set(XWIN_DIR ${xwin_default_dir} CACHE PATH "xwin CRT and Windows SDK directory")
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES LLVM_BIN XWIN_DIR XWIN_ARCH)

set(CMAKE_C_COMPILER "${LLVM_BIN}/clang-cl")
set(CMAKE_CXX_COMPILER "${LLVM_BIN}/clang-cl")
set(CMAKE_C_COMPILER_TARGET ${xwin_target})
set(CMAKE_CXX_COMPILER_TARGET ${xwin_target})
set(CMAKE_LINKER "${LLVM_BIN}/lld-link")
set(CMAKE_RC_COMPILER "${LLVM_BIN}/llvm-rc")
set(CMAKE_MT "${LLVM_BIN}/llvm-mt")
set(CMAKE_AR "${LLVM_BIN}/llvm-lib")

set(xwin_include_flags "")
foreach(include_dir IN ITEMS crt/include sdk/include/ucrt sdk/include/um sdk/include/shared sdk/include/winrt)
    string(APPEND xwin_include_flags " -imsvc\"${XWIN_DIR}/${include_dir}\"")
endforeach()
set(CMAKE_C_FLAGS_INIT "${xwin_include_flags}")
set(CMAKE_CXX_FLAGS_INIT "${xwin_include_flags}")

set(xwin_link_flags "")
foreach(lib_dir IN ITEMS crt/lib/${XWIN_ARCH} sdk/lib/um/${XWIN_ARCH} sdk/lib/ucrt/${XWIN_ARCH})
    string(APPEND xwin_link_flags " /libpath:\"${XWIN_DIR}/${lib_dir}\"")
endforeach()
foreach(target_type IN ITEMS EXE SHARED MODULE)
    set(CMAKE_${target_type}_LINKER_FLAGS_INIT "${xwin_link_flags}")
endforeach()
