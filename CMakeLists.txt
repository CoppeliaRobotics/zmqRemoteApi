cmake_minimum_required(VERSION 3.12)
project(zmqRemoteApi)

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        set(COPPELIASIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    endif()
endif()
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4.5.1.7 REQUIRED)

coppeliasim_add_addon("addOns/ZMQ remote API server.lua")

coppeliasim_add_resource_directory(clients SUBDIR programming/zmqRemoteApi)
coppeliasim_add_resource_directory(tools SUBDIR programming/zmqRemoteApi)
coppeliasim_add_resource(license.txt SUBDIR programming/zmqRemoteApi)
