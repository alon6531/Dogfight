//
// Created by User on 02/04/2026.
//

#include "Engine.h"

#include <iostream>
#include <ostream>

#include "Global.h"


/**
 * Constructor
 */
Engine::Engine() {
    InitWindow(WIN_WIDTH, WIN_HEIGHT, "Dogfight");

    SetTargetFPS(240);

    m_camera = { 0 };
    m_camera.position = { 10.0f, 10.0f, 10.0f };
    m_camera.target = { 0.0f, 0.0f, 0.0f };
    m_camera.up = { 0.0f, 1.0f, 0.0f };
    m_camera.fovy = 45.0f;
    m_camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor();

    std::vector<Obstacle> obstacles;


    Obstacle sphereObs;
    sphereObs.pos = { 50.0f, 0.0f, 0.0f };
    sphereObs.radius = 10.0f;
    //obstacles.push_back(sphereObs);

    m_shouldClose = false;
    m_obstacle = GenMeshSphere(sphereObs.radius, 32, 32);
    m_obstacleModel = LoadModelFromMesh(m_obstacle);



    m_navGraph.BuildGraphFromMap({500, 100, 500}, 10, obstacles);
    m_navGraph.PrepareGPUData();
    m_navGraph.BuildDistanceMatrix();

    m_plane = std::make_unique<Plane>(Vector3(), (Vector3){0, 5, 5}, (Vector3){0,0,0});
}

/**
 * Destructor
 */
Engine::~Engine() {
    CloseWindow();
}

/**
 * Incharge of all the keyboard inputs
 */
void Engine::ProcessInput() {


    if (WindowShouldClose()) m_shouldClose = true;

}

/**
 * Update every tick of the simulator
 * @param deltaTime
 */
void Engine::Update(float deltaTime) {

    UpdateCamera(&m_camera, CAMERA_FREE);




    m_plane->Update(deltaTime, m_navGraph);


    std::cout << m_navGraph.GetHeuristic(0, 600) << std::endl;

}

/**
 * Render every tick of the simulator
 */
void Engine::Render() {
    BeginDrawing();
    ClearBackground(LIGHTGRAY);

    BeginMode3D(m_camera);

    float axisLength = 10.0f;
    float axisThick  = 0.08f;
    int slices = 8;

    DrawCylinderEx({ 0, 0, 0 }, { axisLength, 0, 0 }, axisThick, axisThick, slices, RED);
    DrawCylinderEx({ 0, 0, 0 }, { 0, axisLength, 0 }, axisThick, axisThick, slices, GREEN);
    DrawCylinderEx({ 0, 0, 0 }, { 0, 0, axisLength }, axisThick, axisThick, slices, BLUE);

    DrawModel(m_obstacleModel, { 0, 0, 0 }, 1.0f, GRAY);


    DrawGrid(20, 1.0f);


    m_navGraph.Draw(m_camera.position, 100);

    m_plane->Draw();






    EndMode3D();

    DrawText("X Axis: RED", 10, 70, 20, RED);
    DrawText("Y Axis: GREEN", 10, 95, 20, DARKGREEN);
    DrawText("Z Axis: BLUE", 10, 120, 20, BLUE);


    DrawFPS(10, 10);
    DrawText("ENGINE MODE: ACTIVE", 10, 40, 20, DARKGREEN);
    EndDrawing();
}

void Engine::Run() {
    while (!m_shouldClose) {

        float dt = GetFrameTime();

        ProcessInput();

        Update(dt);

        Render();
    }
}

