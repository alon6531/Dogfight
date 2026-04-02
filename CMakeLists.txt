cmake_minimum_required(VERSION 3.20)

# פתרון ישיר לשגיאה שקיבלת: אומר ל-CMake לאפשר תאימות אחורה לגרסה 3.5
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

project(Dogfight)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

FetchContent_Declare(
        raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/5.0.tar.gz
)

# הגדרות אופטימיזציה ל-Raylib
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(USE_AUDIO OFF CACHE BOOL "" FORCE) # חוסך בעיות סאונד ב-Windows

FetchContent_MakeAvailable(raylib)

add_executable(Dogfight main.cpp)

target_link_libraries(Dogfight raylib)
target_include_directories(Dogfight PUBLIC ${raylib_SOURCE_DIR}/src)