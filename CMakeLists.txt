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
        src/main.cpp
        src/Engine/Engine.cpp src/Engine/Engine.h
        src/Global.h
        src/World/GraphBuilder.h src/World/GraphBuilder.cpp
        src/Entities/Plane/Plane.cpp src/Entities/Plane/Plane.h
        src/Global.cpp
        src/World/Map.cpp
        src/World/Map.h
        src/Navigation/DStarLite.cpp
        src/Navigation/DStarLite.h
        src/Entities/Plane/Control/MPCController.cpp
        src/Entities/Plane/Control/MPCController.h
        src/WindowState/Type/SimsState.cpp
        src/WindowState/Type/SimsState.h
        src/WindowState/Base/State.cpp
        src/WindowState/Base/State.h
        src/WindowState/Type/MenuState.cpp
        src/WindowState/Type/MenuState.h
        src/WindowState/Type/GameOverState.cpp
        src/WindowState/Type/GameOverState.h
        src/Collision/GJK.cpp
        src/Collision/GJK.h
        src/Entities/Plane/AIState/Base/AIState.cpp
        src/Entities/Plane/AIState/Base/AIState.h
        src/Navigation/AStar.cpp
        src/Navigation/AStar.h
        src/Navigation/Dijkstra.cpp
        src/Navigation/Dijkstra.h
        src/Entities/Plane/AIState/Type/PatrolState.cpp
        src/Entities/Plane/AIState/Type/PatrolState.h
        src/Entities/Plane/AIState/Type/TakeOffState.cpp
        src/Entities/Plane/AIState/Type/TakeOffState.h
        src/Entities/Plane/Type/SelfPlane.cpp
        src/Entities/Plane/Type/SelfPlane.h
        src/Entities/Plane/AIState/Type/PursuitState.cpp
        src/Entities/Plane/AIState/Type/PursuitState.h
        src/Entities/Plane/AIState/Type/EvasionState.cpp
        src/Entities/Plane/AIState/Type/EvasionState.h
        src/Entities/Plane/AIState/Type/FuelState.cpp
        src/Entities/Plane/AIState/Type/FuelState.h
)


target_link_libraries(Dogfight PUBLIC raylib)

if (OpenMP_CXX_FOUND)
    target_link_libraries(Dogfight PUBLIC OpenMP::OpenMP_CXX)

    target_link_libraries(Dogfight PUBLIC gomp)
endif()

target_include_directories(Dogfight PUBLIC ${raylib_SOURCE_DIR}/src)

add_definitions(-DNOGDI -DNOUSER -DWIN32_LEAN_AND_MEAN)