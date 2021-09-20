cmake_minimum_required(VERSION 3.5)
project(zmqRemoteApi-client-cpp)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)
set(CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)

include(FetchContent)

FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/ArthurSonzogni/nlohmann_json_cmake_fetchcontent #https://github.com/nlohmann/json
    GIT_TAG v3.9.1
)
FetchContent_GetProperties(json)
if(NOT json_POPULATED)
    FetchContent_Populate(json)
    add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR} EXCLUDE_FROM_ALL)
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

add_library(RemoteAPIClient STATIC RemoteAPIClient.cpp)
target_link_libraries(RemoteAPIClient PUBLIC nlohmann_json::nlohmann_json)
target_link_libraries(RemoteAPIClient PUBLIC cppzmq)

add_executable(example example.cpp)
target_link_libraries(example PRIVATE RemoteAPIClient)

add_executable(synchronousImageTransmission synchronousImageTransmission.cpp)
target_link_libraries(synchronousImageTransmission PRIVATE RemoteAPIClient)
