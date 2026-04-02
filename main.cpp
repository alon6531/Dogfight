#include <raylib.h>

int main() {

    InitWindow(1280, 720, "Dogfight");



    SetTargetFPS(60);
    Camera3D camera = { 0 };
    camera.position = Vector3(10.0f, 10.0f, 10.0f);
    camera.target = Vector3();
    camera.up = Vector3(0.0f, 1.0f, 0.0f);
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor();

    Mesh cube = GenMeshCube(1, 1, 1);
    Model cubeModel = LoadModelFromMesh(cube);


    while (!WindowShouldClose()) {
        UpdateCamera(&camera, CAMERA_FREE);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        DrawGrid(20, 1.0f);


        DrawModel(cubeModel, Vector3(), 1, RED);
        DrawLine3D(Vector3(-4,0,-2), Vector3(5, 2, 3), GREEN);


        EndMode3D();


        DrawFPS(10, 40);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}