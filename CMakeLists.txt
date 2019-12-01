cmake_minimum_required(VERSION 3.7.2)

project(EDynTestbed VERSION 0.0.0 LANGUAGES CXX)

#
# Explicit debug build by default
#

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

#
# Warnings
#

if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wno-reorder -Wno-long-long -Wimplicit-fallthrough)
endif()

#
# Options
#

#
# Dependencies
#

set(EDyn_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../edyn/include;${CMAKE_SOURCE_DIR}/../edyn/build/include CACHE PATH "EDyn include directory.")
find_library(EDyn_LIBRARY EDyn PATHS ${CMAKE_SOURCE_DIR}/../edyn/build/lib)

set(Bx_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../bx/include CACHE PATH "bx include directory.")
set(Bgfx_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../bgfx/include;${CMAKE_SOURCE_DIR}/../bgfx/examples;${CMAKE_SOURCE_DIR}/../bgfx/3rdparty CACHE PATH "bgfx include directory.")
set(Bimg_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../bimg/include CACHE PATH "bimg include directory.")
set(Bgfx_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../bgfx/.build/linux64_gcc/bin CACHE PATH "bgfx library directory.")

find_library(Bx_LIBRARY_RELEASE bxRelease PATHS ${Bgfx_LIBRARY_DIR})
find_library(Bx_LIBRARY_DEBUG bxDebug PATHS ${Bgfx_LIBRARY_DIR})
find_library(Bgfx_LIBRARY_RELEASE bgfxRelease PATHS ${Bgfx_LIBRARY_DIR})
find_library(Bgfx_LIBRARY_DEBUG bgfxDebug PATHS ${Bgfx_LIBRARY_DIR})
find_library(BgfxExampleCommon_LIBRARY_RELEASE example-commonRelease PATHS ${Bgfx_LIBRARY_DIR})
find_library(BgfxExampleCommon_LIBRARY_DEBUG example-commonDebug PATHS ${Bgfx_LIBRARY_DIR})
find_library(BgfxExampleGlue_LIBRARY_RELEASE example-glueRelease PATHS ${Bgfx_LIBRARY_DIR})
find_library(BgfxExampleGlue_LIBRARY_DEBUG example-glueDebug PATHS ${Bgfx_LIBRARY_DIR})
find_library(Bimg_LIBRARY_RELEASE bimgRelease PATHS ${Bgfx_LIBRARY_DIR})
find_library(Bimg_LIBRARY_DEBUG bimgDebug PATHS ${Bgfx_LIBRARY_DIR})
find_library(BimgDecode_LIBRARY_RELEASE bimg_decodeRelease PATHS ${Bgfx_LIBRARY_DIR})
find_library(BimgDecode_LIBRARY_DEBUG bimg_decodeDebug PATHS ${Bgfx_LIBRARY_DIR})
find_library(BimgEncode_LIBRARY_RELEASE bimg_encodeRelease PATHS ${Bgfx_LIBRARY_DIR})
find_library(BimgEncode_LIBRARY_DEBUG bimg_encodeDebug PATHS ${Bgfx_LIBRARY_DIR})

#
# Conan dependencies
#

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

#
# Executable
#

set(EDynTestbed_SOURCES
    src/main.cpp
    src/debugdraw.cpp
    src/edyn_example.cpp
    src/particles.cpp
    src/restitution.cpp
    src/cradle.cpp
)

add_executable(EDynTestbed ${EDynTestbed_SOURCES})

target_include_directories(EDynTestbed
    PUBLIC ${CMAKE_SOURCE_DIR}/include
)

target_include_directories(EDynTestbed SYSTEM
    PUBLIC ${EDyn_INCLUDE_DIR}
    PUBLIC ${CONAN_INCLUDE_DIRS_ENTT}
    PUBLIC ${Bx_INCLUDE_DIR}
    PUBLIC ${Bgfx_INCLUDE_DIR}
    PUBLIC ${Bimg_INCLUDE_DIR}
)

target_compile_features(EDynTestbed PUBLIC cxx_std_17)

target_link_libraries(EDynTestbed
    optimized ${BgfxExampleGlue_LIBRARY_RELEASE}
    debug ${BgfxExampleGlue_LIBRARY_DEBUG}
    optimized ${BgfxExampleCommon_LIBRARY_RELEASE}
    debug ${BgfxExampleCommon_LIBRARY_DEBUG}
    optimized ${BimgDecode_LIBRARY_RELEASE}
    debug ${BimgDecode_LIBRARY_DEBUG}
    optimized ${BimgEncode_LIBRARY_RELEASE}
    debug ${BimgEncode_LIBRARY_DEBUG}
    optimized ${Bimg_LIBRARY_RELEASE}
    debug ${Bimg_LIBRARY_DEBUG}
    optimized ${Bgfx_LIBRARY_RELEASE}
    debug ${Bgfx_LIBRARY_DEBUG}
    optimized ${Bx_LIBRARY_RELEASE}
    debug ${Bx_LIBRARY_DEBUG}
    ${EDyn_LIBRARY}
)

if (UNIX AND NOT APPLE)
    target_link_libraries(EDynTestbed
        dl
        pthread
        X11
        GL
    )
endif ()
