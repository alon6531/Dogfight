#include <raylib.h>

int main() {

    InitWindow(1280, 720, "Air Combat - D* Lite Visualization");



    SetTargetFPS(60);

    while (!WindowShouldClose()) {


        BeginDrawing();
        ClearBackground(RAYWHITE);



        DrawText("Use WASD and Mouse to move camera", 10, 10, 20, DARKGRAY);
        DrawFPS(10, 40);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}