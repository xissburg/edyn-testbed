cmake_minimum_required(VERSION 3.7.2)

project(EdynTestbed VERSION 1.0.0 LANGUAGES CXX)

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
#find_package(Edyn REQUIRED)

set(Edyn_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../edyn/include;${CMAKE_SOURCE_DIR}/../edyn/build/include CACHE PATH "Edyn include directory.")
set(Edyn_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../edyn/build/lib CACHE PATH "Edyn library directory.")
find_library(Edyn_LIBRARY Edyn PATHS ${Edyn_LIBRARY_DIR} PATH_SUFFIXES Debug Release RelWithDebInfo)
find_library(Edyn_LIBRARY_DEBUG Edyn_d PATHS ${Edyn_LIBRARY_DIR} PATH_SUFFIXES Debug Release RelWithDebInfo)

if (EDYN_BUILD_CLIENT)
    add_subdirectory(client)
endif ()

if (EDYN_BUILD_SERVER)
    add_subdirectory(server)
endif ()
