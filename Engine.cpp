//
// Created by User on 02/04/2026.
//

#include "Engine.h"

#include <iostream>
#include <ostream>

#include "Global.h"
#include "raymath.h"


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




    Obstacle sphereObs;
    sphereObs.pos = {-240, 0, -150};
    sphereObs.radius = 5.0f;
    m_obstacles.push_back(sphereObs);

    Obstacle sphereObs1;
    sphereObs1.pos = {-240, 0, -135};
    sphereObs1.radius = 15.0f;
    m_obstacles.push_back(sphereObs1);

    m_shouldClose = false;



    Vector3 mapSize = {500, 100, 500};
    m_navGraph.BuildGraphFromMap(mapSize, 10, m_obstacles);
    m_navGraph.PrepareGPUData();
    m_navGraph.BuildDistanceMatrix();



    int startNodeIdx = m_navGraph.nodes().size() / 2;

    m_plane = std::make_unique<Plane>(m_navGraph.nodes()[startNodeIdx].position, (Vector3){0, 5, 5}, (Vector3){0,0,0});
    m_plane->SetDestination(10, m_navGraph);





    Vector3 heightMapSize = Vector3(mapSize.x * 5, mapSize.y *5, mapSize.z * 5);
    m_map.Load("HeightMap.png", heightMapSize  , "MapTexture.png");
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
    UpdateCamera(&m_camera, CAMERA_FREE);
    UpdateCamera(&m_camera, CAMERA_FREE);

    m_map.UpdateFog(m_camera.position);

    std::cout << m_camera.position.x << " " <<m_camera.position.z << std::endl;



    m_plane->Update(deltaTime);





}

/**
 * Render every tick of the simulator
 */
void Engine::Render() {
    BeginDrawing();
    ClearBackground(SKYBLUE);

    BeginMode3D(m_camera);

    float axisLength = 10.0f;
    float axisThick  = 0.08f;
    int slices = 8;

    DrawCylinderEx({ 0, 0, 0 }, { axisLength, 0, 0 }, axisThick, axisThick, slices, RED);
    DrawCylinderEx({ 0, 0, 0 }, { 0, axisLength, 0 }, axisThick, axisThick, slices, GREEN);
    DrawCylinderEx({ 0, 0, 0 }, { 0, 0, axisLength }, axisThick, axisThick, slices, BLUE);

    for (auto obstacle: m_obstacles) {
        DrawSphere(obstacle.pos, obstacle.radius, GRAY);
    }



    DrawSphere(m_navGraph.nodes()[10].position, 1, GREEN);




    DrawGrid(20, 1.0f);


    m_navGraph.Draw(m_camera.position, 100);

    m_plane->Draw();


    m_map.Draw();



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

