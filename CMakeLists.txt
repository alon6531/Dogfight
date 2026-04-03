cmake_minimum_required(VERSION 3.20)


set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

project(Dogfight)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

FetchContent_Declare(
        raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/5.0.tar.gz
)


set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(USE_AUDIO OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(raylib)

add_executable(Dogfight main.cpp
        Engine.cpp
        Engine.h
        Global.h
        GraphBuilder.h
        GraphBuilder.cpp)

target_link_libraries(Dogfight raylib)
target_include_directories(Dogfight PUBLIC ${raylib_SOURCE_DIR}/src)