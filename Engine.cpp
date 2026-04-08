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

void Engine::InitializeSystem() {
    Obstacle sphereObs;
    sphereObs.pos = {-240, 0, -150};
    sphereObs.radius = 5.0f;
    m_obstacles.push_back(sphereObs);

    Obstacle sphereObs1;
    sphereObs1.pos = {170, -100, 170};
    sphereObs1.radius = 70.0f;
    m_obstacles.push_back(sphereObs1);



    m_shouldClose = false;

    Vector3 fullMapSize = { 1500, 750, 1500 };
    m_map.Load("HeightMap.png", fullMapSize  , "MapTexture.png");




    m_navGraph.BuildGraphFromMap(fullMapSize, 50, m_obstacles, m_map);
    m_navGraph.PrepareGPUData();
    m_navGraph.BuildDistanceMatrix();


    int startNodeIdx = m_navGraph.GetClosestNode({600, -270, 200});
    m_starPoint = m_navGraph.nodes()[startNodeIdx].position;

    int targetNodeIdx = m_navGraph.GetClosestNode({100, -24, 50});
    m_targetPoint = m_navGraph.nodes()[targetNodeIdx].position;

    m_camera.position = { m_starPoint.x, m_starPoint.y + 50, m_starPoint.z - 100 };
    m_camera.target = m_starPoint;

    m_plane = std::make_unique<Plane>(m_starPoint, Vector3(5, 5, 5), Vector3(0, 0, 0), PINK, m_navGraph);
    m_enemy = std::make_unique<Plane>(Vector3(-100, -120, -150), Vector3(), Vector3(), YELLOW, m_navGraph);
    m_fsm = std::make_unique<FSM>(m_navGraph);
    m_mpcController = std::make_unique<MPCController>((MPCParameters){12, 0.15f, 50.0f, 2.5f, 1.0f});




}

/**
 * Update every tick of the simulator
 * @param deltaTime
 */
void Engine::Update(float deltaTime) {

    UpdateCamera(&m_camera, CAMERA_FREE);
    UpdateCamera(&m_camera, CAMERA_FREE);
    UpdateCamera(&m_camera, CAMERA_FREE);

    // Vector3 planePos = m_plane->position();
    // m_camera.target = planePos;
    // m_camera.position = Vector3Add(planePos, {0, 30, -60});

    m_fsm->Update(*m_plane, *m_enemy, m_targetPoint, deltaTime);


    m_map.UpdateFog(m_camera.position);
    m_plane->Update(m_enemy->position(), deltaTime, *m_mpcController);
    m_enemy->Update(m_plane->position(), deltaTime, *m_mpcController);

    //std::cout << "Camera pos: " <<  m_camera.position.x << " " <<  m_camera.position.y << " " <<  m_camera.position.z << std::endl;





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


    DrawCube(m_starPoint, 10, 2, 10, GREEN);
    DrawSphere(m_targetPoint, 1, GREEN);




    DrawGrid(20, 1.0f);


    m_navGraph.Draw(m_camera.position, 100);

    m_plane->Draw();
    m_enemy->Draw();

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

    this->InitializeSystem();





    while (!m_shouldClose) {

        float dt = GetFrameTime();

        ProcessInput();

        Update(dt);

        Render();
    }
}

