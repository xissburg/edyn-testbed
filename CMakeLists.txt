cmake_minimum_required(VERSION 3.23.0)

project(EdynTestbed VERSION 1.3.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
  add_compile_options(/W4 /bigobj)
else()
  add_compile_options(-Wall -Wno-reorder -Wno-long-long -Wimplicit-fallthrough)
endif()

#
# Options
#

option(EDYN_BUILD_NETWORKING_EXAMPLE "Build networking example" OFF)
option(EDYN_SOUND_ENABLED "Enable sounds with SoLoud" OFF)
option(EDYN_BUILD_CLIENT "Disable if building in a server environment." ON)
option(EDYN_BUILD_SERVER "Build servers for networking examples." OFF)

#
# Dependencies
#

find_package(EnTT REQUIRED)
find_package(Edyn REQUIRED)

if (EDYN_BUILD_CLIENT)
    add_subdirectory(client)
endif ()

if (EDYN_BUILD_SERVER)
    add_subdirectory(server)
endif ()
