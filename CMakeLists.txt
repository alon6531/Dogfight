cmake_minimum_required(VERSION 3.20)
project(Dogfight)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

find_package(OpenMP REQUIRED)


if(OpenMP_CXX_FOUND)
    add_compile_options(-fopenmp)

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fopenmp")
endif()

include(FetchContent)
FetchContent_Declare(
        raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/5.0.tar.gz
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(USE_AUDIO OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)


add_executable(Dogfight
        main.cpp
        Engine.cpp Engine.h
        Global.h
        GraphBuilder.h GraphBuilder.cpp
        Plane.cpp Plane.h
        Global.cpp
        Map.cpp
        Map.h
        FSM.cpp
        FSM.h
        DStarLite.cpp
        DStarLite.h
        MPCController.cpp
        MPCController.h
)


target_link_libraries(Dogfight PUBLIC raylib)

if (OpenMP_CXX_FOUND)
    target_link_libraries(Dogfight PUBLIC OpenMP::OpenMP_CXX)

    target_link_libraries(Dogfight PUBLIC gomp)
endif()

target_include_directories(Dogfight PUBLIC ${raylib_SOURCE_DIR}/src)