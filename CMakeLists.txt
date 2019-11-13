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
)

add_executable(EDynTestbed ${EDynTestbed_SOURCES})

target_include_directories(EDynTestbed SYSTEM
    PUBLIC ${EDyn_INCLUDE_DIR}
)

target_include_directories(EDynTestbed SYSTEM 
    PUBLIC ${CONAN_INCLUDE_DIRS_ENTT}
)

target_compile_features(EDynTestbed PUBLIC cxx_std_17)

target_link_libraries(EDynTestbed
    PRIVATE ${EDyn_LIBRARY}
)

if (UNIX AND NOT APPLE)
    target_link_libraries(EDynTestbed PRIVATE
        dl
        pthread
    )
endif ()
