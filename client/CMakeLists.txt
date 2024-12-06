# Bgfx

set(PROJECTS_DIR ${CMAKE_SOURCE_DIR}/..)
set(Bx_INCLUDE_DIR ${PROJECTS_DIR}/bx/include CACHE PATH "bx include directory.")
set(Bgfx_INCLUDE_DIR ${PROJECTS_DIR}/bgfx/include;${PROJECTS_DIR}/bgfx/examples;${PROJECTS_DIR}/bgfx/3rdparty CACHE PATH "bgfx include directory.")
set(Bimg_INCLUDE_DIR ${PROJECTS_DIR}/bimg/include CACHE PATH "bimg include directory.")
set(Bgfx_LIBRARY_DIR ${PROJECTS_DIR}/bgfx/.build CACHE PATH "bgfx library directory.")

find_package(Taskflow)
find_package(enkits)
find_library(enkiTS_LIBRARY enkiTS PATHS ${enkits_LIB_DIRS_RELEASE})

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

# SoLoud

if (EDYN_SOUND_ENABLED)
    set(SoLoud_INCLUDE_DIR ${PROJECTS_DIR}/soloud/include CACHE PATH "SoLoud include directory.")
    set(SoLoud_LIBRARY_DIR ${PROJECTS_DIR}/soloud/lib CACHE PATH "SoLoud library directory.")
    find_library(SoLoud_LIBRARY_RELEASE soloud_static PATHS ${SoLoud_LIBRARY_DIR})
    find_library(SoLoud_LIBRARY_DEBUG soloud_static_x64_d PATHS ${SoLoud_LIBRARY_DIR})

    find_package(SDL2 REQUIRED)
endif()

# Collect source files.

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
    src/center_of_mass.cpp
    src/raycasting.cpp
    src/friction.cpp
    src/billiards.cpp
    src/per_vertex_materials.cpp
    src/cvjoint.cpp
    src/cone_constraint.cpp
    src/ragdoll.cpp
    src/generic_constraint.cpp
    src/sensors.cpp
    src/vehicle.cpp
    src/curb.cpp
    src/hover.cpp
    src/rigidbody_kind.cpp
    src/shape_shifting.cpp
    src/soft_contacts.cpp
    src/taskflow.cpp
    src/enkits.cpp
    ${CMAKE_SOURCE_DIR}/common/src/vehicle_system.cpp
)

if (EDYN_BUILD_NETWORKING_EXAMPLE)
    find_library(ENet_LIBRARY enet)

    set(EdynTestbed_SOURCES
        ${EdynTestbed_SOURCES}
        src/vehicle_networking.cpp
        src/networking.cpp
        src/register_networked_components.cpp)
endif ()

# Executable

if (APPLE)
    add_executable(EdynTestbed MACOSX_BUNDLE ${EdynTestbed_SOURCES})
elseif (WIN32)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
    add_executable(EdynTestbed WIN32 ${EdynTestbed_SOURCES})
else ()
    add_executable(EdynTestbed ${EdynTestbed_SOURCES})
endif ()

# Assign include directories.

target_include_directories(EdynTestbed
    PUBLIC ${CMAKE_SOURCE_DIR}/client/include
    PUBLIC ${CMAKE_SOURCE_DIR}/common/include
)

target_include_directories(EdynTestbed SYSTEM
    PUBLIC ${Edyn_INCLUDE_DIR}
    PUBLIC ${CONAN_INCLUDE_DIRS_ENTT}
    PUBLIC ${enkits_INCLUDE_DIRS}
    PUBLIC ${Bx_INCLUDE_DIR}
    PUBLIC ${Bgfx_INCLUDE_DIR}
    PUBLIC ${Bimg_INCLUDE_DIR}
)

if (EDYN_SOUND_ENABLED)
    target_include_directories(EdynTestbed SYSTEM
        PUBLIC ${SoLoud_INCLUDE_DIR}
        PUBLIC ${SDL2_INCLUDE_DIRS}
    )
    target_compile_definitions(EdynTestbed PRIVATE
        EDYN_SOUND_ENABLED
    )
endif ()

target_compile_features(EdynTestbed PUBLIC cxx_std_17)

# Link libraries.

target_link_libraries(EdynTestbed
    EnTT::EnTT
    Edyn::Edyn
)

target_link_libraries(EdynTestbed
    Taskflow::Taskflow
    ${enkiTS_LIBRARY}
)

if (EDYN_BUILD_NETWORKING_EXAMPLE)
    target_include_directories(EdynTestbed SYSTEM
        PUBLIC ${ENet_INCLUDE_DIR}
    )
    target_link_libraries(EdynTestbed
        ${ENet_LIBRARY}
    )
endif ()

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
)

if (EDYN_SOUND_ENABLED)
    target_link_libraries(EdynTestbed
        optimized ${SoLoud_LIBRARY_RELEASE}
        debug ${SoLoud_LIBRARY_DEBUG}
        ${SDL2_LIBRARIES}
    )
endif ()

if (APPLE)
    set_target_properties(EdynTestbed PROPERTIES
        LINK_FLAGS "-framework Cocoa -framework Metal -framework QuartzCore")
endif ()

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
    target_link_libraries(EdynTestbed winmm Ws2_32)
endif ()
