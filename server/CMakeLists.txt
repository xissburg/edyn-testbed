function(make_server ProjectName)
    project(${ProjectName} VERSION 0.0.0 LANGUAGES CXX)

    # Executable.
    add_executable(${ProjectName} ${ARGN})

    # Assign include directories.
    target_include_directories(${ProjectName}
        PUBLIC ${CMAKE_SOURCE_DIR}/server/include
        PUBLIC ${CMAKE_SOURCE_DIR}/common/include
    )

    target_include_directories(${ProjectName} SYSTEM
        PUBLIC ${ENet_INCLUDE_DIR}
    )

    target_compile_features(${ProjectName} PUBLIC cxx_std_17)

    # Link libraries.
    target_link_libraries(${ProjectName}
        EnTT::EnTT
        Edyn::Edyn
        ${ENet_LIBRARY}
    )

    if (UNIX AND NOT APPLE)
        target_link_libraries(${ProjectName}
            pthread
        )
    endif ()

    if (WIN32)
        target_link_libraries(${ProjectName} winmm Ws2_32)
    endif ()
endfunction()

make_server(EdynTestbedNetworkingServer
    src/networking_server.cpp;src/edyn_server.cpp)
make_server(EdynTestbedVehicleServer
    src/edyn_server.cpp;src/vehicle_server.cpp;${CMAKE_SOURCE_DIR}/common/src/vehicle_system.cpp)
