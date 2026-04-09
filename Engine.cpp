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
    // ... (הגדרת המכשולים נשארת אותו דבר) ...

    m_shouldClose = false;
    Vector3 fullMapSize = { 1500, 750, 1500 };
    m_map.Load("HeightMap.png", fullMapSize, "MapTexture.png");

    m_navGraph.BuildGraphFromMap(fullMapSize, 50, m_obstacles, m_map);
    m_navGraph.PrepareGPUData();
    m_navGraph.BuildDistanceMatrix();

    // --- לוגיקת רנדומיזציה ---
    std::random_device rd;
    std::mt19937 gen(rd());

    // נגדיר טווחים בהתאם ל-fullMapSize (ממרכז המפה)
    // הערה: אני משתמש ב-0.8 כדי שהם לא ייוולדו ממש על הקצה
    std::uniform_real_distribution<float> distX(-fullMapSize.x * 0.4f, fullMapSize.x * 0.4f);
    std::uniform_real_distribution<float> distZ(-fullMapSize.z * 0.4f, fullMapSize.z * 0.4f);
    std::uniform_real_distribution<float> distY(-100.0f, 200.0f); // גובה טיסה סביר

    // יצירת מיקום רנדומלי למטוס שלנו
    Vector3 randomStartPos = { distX(gen), distY(gen), distZ(gen) };
    int startNodeIdx = m_navGraph.GetClosestNode(randomStartPos);
    m_starPoint = m_navGraph.nodes()[startNodeIdx].position;

    // יצירת מיקום רנדומלי לאויב (מוודאים שהוא לא בדיוק על המטוס שלנו)
    Vector3 randomEnemyPos = { distX(gen), distY(gen), distZ(gen) };
    int enemyNodeIdx = m_navGraph.GetClosestNode(randomEnemyPos);
    Vector3 enemyStartPos = m_navGraph.nodes()[enemyNodeIdx].position;

    // יצירת מטרה רנדומלית (Target Point)
    Vector3 randomTargetPos = { distX(gen), distY(gen), distZ(gen) };
    int targetNodeIdx = m_navGraph.GetClosestNode(randomTargetPos);
    m_targetPoint = m_navGraph.nodes()[targetNodeIdx].position;

    // --- אתחול האובייקטים עם המיקומים החדשים ---
    m_plane = std::make_unique<Plane>(m_starPoint, Vector3{10, 0, 10}, Vector3{0, 0, 0}, PINK, m_navGraph);
    m_enemy = std::make_unique<Plane>(enemyStartPos, Vector3{-10, 0, -10}, Vector3{0, 0, 0}, YELLOW, m_navGraph);

    // הגדרת המצלמה שתסתכל על המטוס שלנו במיקומו החדש
    m_camera.position = { m_starPoint.x, m_starPoint.y + 50, m_starPoint.z - 50 };
    m_camera.target = m_starPoint;

    m_fsm = std::make_unique<FSM>(m_navGraph);
    m_mpcController = std::make_unique<MPCController>((MPCParameters){
        15, 0.05f, 100.0f, 2.5f, 0.8f
    });
}

/**
 * Update every tick of the simulator
 * @param deltaTime
 */
void Engine::Update(float deltaTime) {
    deltaTime = fminf(deltaTime, 0.05f);
    if (deltaTime > 0.05f) deltaTime = 0.05f;

    UpdateCamera(&m_camera, CAMERA_FREE);
    UpdateCamera(&m_camera, CAMERA_FREE);
    UpdateCamera(&m_camera, CAMERA_FREE);
    UpdateCamera(&m_camera, CAMERA_FREE);
    UpdateCamera(&m_camera, CAMERA_FREE);
    UpdateCamera(&m_camera, CAMERA_FREE);

    // Vector3 planePos = m_plane->GetPosition();
    // Vector3 planeVel = m_plane->GetVelocity();
    // Vector3 forward = (Vector3LengthSqr(planeVel) > 0.1f) ? Vector3Normalize(planeVel) : Vector3{ 0, 0, 1 };
    // float distanceBehind = 60.0f;
    // float heightAbove = 25.0f;
    // Vector3 offset = Vector3Scale(forward, -distanceBehind);
    // offset.y += heightAbove;
    // Vector3 targetCameraPos = Vector3Add(planePos, offset);
    // m_camera.position = Vector3Lerp(m_camera.position, targetCameraPos, 10.0f * deltaTime);
    // m_camera.target = planePos;
    //



    m_shouldClose = m_fsm->Update(*m_plane, *m_enemy, m_targetPoint, deltaTime);


    m_map.UpdateFog(m_camera.position);
    m_plane->Update(*m_enemy, deltaTime, *m_mpcController);
    m_enemy->Update(*m_plane, deltaTime, *m_mpcController);

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

    m_plane->Draw(m_camera);
    m_enemy->Draw(m_camera);

    m_map.Draw();



    EndMode3D();

    DrawText("X Axis: RED", 10, 70, 20, RED);
    DrawText("Y Axis: GREEN", 10, 95, 20, DARKGREEN);
    DrawText("Z Axis: BLUE", 10, 120, 20, BLUE);

    DrawText(TextFormat("Enemy Pos: %.2f, %.2f, %.2f",
             m_enemy->GetPosition().x, m_enemy->GetPosition().y, m_enemy->GetPosition().z),
             10, 150, 20, YELLOW);

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

