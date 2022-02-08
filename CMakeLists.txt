cmake_minimum_required(VERSION 3.7.2)

project(EdynTestbed VERSION 0.0.0 LANGUAGES CXX)

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
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wno-reorder -Wno-long-long -Wimplicit-fallthrough)
endif()

#
# Options
#

option(EDYN_BUILD_NETWORKING_EXAMPLE "Build networking example" OFF)
option(EDYN_SOUND_ENABLED "Enable sounds with SoLoud" OFF)

#
# Dependencies
#

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
include(${CMAKE_BINARY_DIR}/conan_paths.cmake OPTIONAL)
conan_basic_setup()

find_package(EnTT REQUIRED)
#find_package(Edyn REQUIRED)

set(Edyn_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../edyn/include;${CMAKE_SOURCE_DIR}/../edyn/build/include CACHE PATH "Edyn include directory.")
set(Edyn_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../edyn/build/lib CACHE PATH "Edyn library directory.")
find_library(Edyn_LIBRARY Edyn PATHS ${Edyn_LIBRARY_DIR} PATH_SUFFIXES Debug Release RelWithDebInfo)
find_library(Edyn_LIBRARY_DEBUG Edyn_d PATHS ${Edyn_LIBRARY_DIR} PATH_SUFFIXES Debug Release RelWithDebInfo)

set(Bx_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../bx/include CACHE PATH "bx include directory.")
set(Bgfx_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../bgfx/include;${CMAKE_SOURCE_DIR}/../bgfx/examples;${CMAKE_SOURCE_DIR}/../bgfx/3rdparty CACHE PATH "bgfx include directory.")
set(Bimg_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../bimg/include CACHE PATH "bimg include directory.")
set(Bgfx_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../bgfx/.build CACHE PATH "bgfx library directory.")

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

if(EDYN_SOUND_ENABLED)
    set(SoLoud_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../soloud/include CACHE PATH "SoLoud include directory.")
    set(SoLoud_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../soloud/lib CACHE PATH "SoLoud library directory.")
    find_library(SoLoud_LIBRARY_RELEASE soloud_static PATHS ${SoLoud_LIBRARY_DIR})
    find_library(SoLoud_LIBRARY_DEBUG soloud_static_x64_d PATHS ${SoLoud_LIBRARY_DIR})

    find_package(SDL2 REQUIRED)
endif()

#
# Executable
#

set(EdynTestbed_SOURCES
    src/main.cpp
    src/debugdraw.cpp
    src/edyn_example.cpp
    src/particles.cpp
    src/polyhedrons.cpp
    src/restitution.cpp
    src/cradle.cpp
    src/cylinders.cpp
    src/capsules.cpp
    src/hinge.cpp
    src/triangle_mesh.cpp
    src/paged_triangle_mesh.cpp
    src/boxes.cpp
    src/compound.cpp
    src/platforms.cpp
    src/external_systems.cpp
    src/center_of_mass.cpp
    src/raycasting.cpp
    src/friction.cpp
    src/billiards.cpp
    src/per_vertex_materials.cpp
    src/cvjoint.cpp
    src/cone_constraint.cpp
    src/ragdoll.cpp
)

if(EDYN_BUILD_NETWORKING_EXAMPLE)
    find_library(ENet_LIBRARY enet)

    set(EdynTestbed_SOURCES
        ${EdynTestbed_SOURCES}
        src/networking.cpp
        src/register_networked_components.cpp)
endif()

if (APPLE)
	add_executable(EdynTestbed MACOSX_BUNDLE ${EdynTestbed_SOURCES})
elseif (WIN32)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
	add_executable(EdynTestbed WIN32 ${EdynTestbed_SOURCES})
else ()
	add_executable(EdynTestbed ${EdynTestbed_SOURCES})
endif ()

target_include_directories(EdynTestbed
    PUBLIC ${CMAKE_SOURCE_DIR}/include
)

target_include_directories(EdynTestbed SYSTEM
    PUBLIC ${Edyn_INCLUDE_DIR}
    PUBLIC ${CONAN_INCLUDE_DIRS_ENTT}
    PUBLIC ${Bx_INCLUDE_DIR}
    PUBLIC ${Bgfx_INCLUDE_DIR}
    PUBLIC ${Bimg_INCLUDE_DIR}
)

if(EDYN_SOUND_ENABLED)
    target_include_directories(EdynTestbed SYSTEM
        PUBLIC ${SoLoud_INCLUDE_DIR}
        PUBLIC ${SDL2_INCLUDE_DIRS}
    )
    target_compile_definitions(EdynTestbed PRIVATE
        EDYN_SOUND_ENABLED
    )
endif()

target_compile_features(EdynTestbed PUBLIC cxx_std_17)

# Sanitize

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    #target_compile_options(EdynTestbed PUBLIC -fsanitize=address -fno-omit-frame-pointer -fsanitize=undefined)
endif()

# Link libraries

target_link_libraries(EdynTestbed
    optimized ${BgfxExampleGlue_LIBRARY_RELEASE}
    debug ${BgfxExampleGlue_LIBRARY_DEBUG}
    optimized ${BgfxExampleCommon_LIBRARY_RELEASE}
    debug ${BgfxExampleCommon_LIBRARY_DEBUG}
    optimized ${BimgDecode_LIBRARY_RELEASE}
    debug ${BimgDecode_LIBRARY_DEBUG}
    optimized ${Bimg_LIBRARY_RELEASE}
    debug ${Bimg_LIBRARY_DEBUG}
    optimized ${Bgfx_LIBRARY_RELEASE}
    debug ${Bgfx_LIBRARY_DEBUG}
    optimized ${Bx_LIBRARY_RELEASE}
    debug ${Bx_LIBRARY_DEBUG}
    EnTT::EnTT
    optimized ${Edyn_LIBRARY}
    debug ${Edyn_LIBRARY_DEBUG}
    #-fsanitize=address
    #-fsanitize=undefined
    )

if(EDYN_BUILD_NETWORKING_EXAMPLE)
    target_link_libraries(EdynTestbed
        ${ENet_LIBRARY}
    )
endif()

if(EDYN_SOUND_ENABLED)
    target_link_libraries(EdynTestbed
        optimized ${SoLoud_LIBRARY_RELEASE}
        debug ${SoLoud_LIBRARY_DEBUG}
        ${SDL2_LIBRARIES}
    )
endif()

if (APPLE)
    set_target_properties(EdynTestbed PROPERTIES
        LINK_FLAGS "-framework Cocoa -framework Metal -framework QuartzCore")
endif()

if (UNIX AND NOT APPLE)
    target_link_libraries(EdynTestbed
        dl
        pthread
        X11
        GL
    )

    if(EDYN_SOUND_ENABLED)
        target_link_libraries(EdynTestbed
            asound
        )
    endif ()
endif ()

if (WIN32)
    target_link_libraries(EdynTestbed winmm)
endif ()

# Server

if(EDYN_BUILD_NETWORKING_EXAMPLE)
    project(EdynTestbedServer VERSION 0.0.0 LANGUAGES CXX)

    set(EdynTestbedServer_SOURCES
        src/server/main.cpp)

	add_executable(EdynTestbedServer ${EdynTestbedServer_SOURCES})

    target_include_directories(EdynTestbedServer
        PUBLIC ${CMAKE_SOURCE_DIR}/include
    )

    target_include_directories(EdynTestbedServer SYSTEM
        PUBLIC ${Edyn_INCLUDE_DIR}
        PUBLIC ${CONAN_INCLUDE_DIRS_ENTT}
    )

    target_compile_features(EdynTestbedServer PUBLIC cxx_std_17)

    # Link libraries

    target_link_libraries(EdynTestbedServer
        EnTT::EnTT
        optimized ${Edyn_LIBRARY}
        debug ${Edyn_LIBRARY_DEBUG}
        ${ENet_LIBRARY}
    )

    if (UNIX AND NOT APPLE)
        target_link_libraries(EdynTestbedServer
            dl
            pthread
        )
    endif ()

    if (WIN32)
        target_link_libraries(EdynTestbedServer winmm)
    endif ()
endif()
