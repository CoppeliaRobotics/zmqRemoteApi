cmake_minimum_required(VERSION 3.5)
project(zmqRemoteApi-client-cpp3)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)

include(FetchContent)

FetchContent_Declare(jsoncons
    GIT_REPOSITORY https://github.com/danielaparker/jsoncons
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/jsoncons
)
FetchContent_GetProperties(jsoncons)
if(NOT jsoncons_POPULATED)
    FetchContent_Populate(jsoncons)
    #add_subdirectory(${jsoncons_SOURCE_DIR} ${jsoncons_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

FetchContent_Declare(cppzmq
    GIT_REPOSITORY https://github.com/zeromq/cppzmq
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cppzmq
)
FetchContent_GetProperties(cppzmq)
if(NOT cppzmq_POPULATED)
    FetchContent_Populate(cppzmq)
    add_subdirectory(${cppzmq_SOURCE_DIR} ${cppzmq_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

add_executable(client client.cpp)
target_include_directories(client PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/jsoncons/include)
target_link_libraries(client PRIVATE Eigen3::Eigen)
target_link_libraries(client PRIVATE cppzmq)
