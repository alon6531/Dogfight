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
        Sims/GraphBuilder.h Sims/GraphBuilder.cpp
        Sims/Plane.cpp Sims/Plane.h
        Global.cpp
        Sims/Map.cpp
        Sims/Map.h
        Sims/FSM.cpp
        Sims/FSM.h
        Sims/DStarLite.cpp
        Sims/DStarLite.h
        Sims/MPCController.cpp
        Sims/MPCController.h
        State/SimsState.cpp
        State/SimsState.h
        State/State.cpp
        State/State.h
        State/MenuState.cpp
        State/MenuState.h
        State/GameOverState.cpp
        State/GameOverState.h
)


target_link_libraries(Dogfight PUBLIC raylib)

if (OpenMP_CXX_FOUND)
    target_link_libraries(Dogfight PUBLIC OpenMP::OpenMP_CXX)

    target_link_libraries(Dogfight PUBLIC gomp)
endif()

target_include_directories(Dogfight PUBLIC ${raylib_SOURCE_DIR}/src)

add_definitions(-DNOGDI -DNOUSER -DWIN32_LEAN_AND_MEAN)