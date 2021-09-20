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

add_library(RemoteAPIClient STATIC RemoteAPIClient.cpp)
target_include_directories(RemoteAPIClient PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/jsoncons/include)
target_link_libraries(RemoteAPIClient PUBLIC cppzmq)

add_executable(example example.cpp)
target_link_libraries(example PRIVATE RemoteAPIClient)

add_executable(synchronousImageTransmission synchronousImageTransmission.cpp)
target_link_libraries(synchronousImageTransmission PRIVATE RemoteAPIClient)

find_package(OpenCV COMPONENTS core imgproc highgui)
if(OpenCV_FOUND)
    add_executable(opencv opencv.cpp)
    target_include_directories(opencv PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(opencv PRIVATE RemoteAPIClient)
    target_link_libraries(opencv PRIVATE ${OpenCV_LIBS})
endif()

find_package(Qt5 COMPONENTS Core Widgets Gui)
if(Qt5_FOUND)
    add_executable(qt qt.cpp)
    target_link_libraries(qt PRIVATE RemoteAPIClient)
    target_link_libraries(qt PRIVATE Qt5::Widgets)
endif()
