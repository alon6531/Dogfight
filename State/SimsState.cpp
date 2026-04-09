//
// Created by User on 09/04/2026.
//

#include "SimsState.h"

#include "raymath.h"

SimsState::SimsState() : State(), m_map(), m_starPoint(), m_targetPoint() {
    m_camera = {0};
    m_camera.position = {10.0f, 10.0f, 10.0f};
    m_camera.target = {0.0f, 0.0f, 0.0f};
    m_camera.up = {0.0f, 1.0f, 0.0f};
    m_camera.fovy = 45.0f;
    m_camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor();

    InitializeSystem();
}

StateType SimsState::Update(float deltaTime) {

    if (gCameraMode) {
        Vector3 planePos = m_plane->GetPosition();
        Vector3 planeVel = m_plane->GetVelocity();
        Vector3 forward = (Vector3LengthSqr(planeVel) > 0.1f) ? Vector3Normalize(planeVel) : Vector3{ 0, 0, 1 };
        float distanceBehind = 60.0f;
        float heightAbove = 25.0f;
        Vector3 offset = Vector3Scale(forward, -distanceBehind);
        offset.y += heightAbove;
        Vector3 targetCameraPos = Vector3Add(planePos, offset);
        m_camera.position = Vector3Lerp(m_camera.position, targetCameraPos, 10.0f * deltaTime);
        m_camera.target = planePos;
    }
    else {
        int cameraSpeedMult = IsKeyDown(KEY_LEFT_SHIFT) ? 100 : 10;

        for (int i = 0; i < cameraSpeedMult; i++) {
            UpdateCamera(&m_camera, CAMERA_FREE);
        }


    }



    if (m_fsm->Update(*m_plane, *m_enemy, m_targetPoint, deltaTime))
        return StateType::GAME_OVER;


    m_map.UpdateFog(m_camera.position);
    m_plane->Update(*m_enemy, deltaTime, *m_mpcController);
    m_enemy->Update(*m_plane, deltaTime, *m_mpcController);

    //std::cout << "Camera pos: " <<  m_camera.position.x << " " <<  m_camera.position.y << " " <<  m_camera.position.z << std::endl;

    return StateType::SIMULATION;


}

void SimsState::InitializeSystem() {
    Vector3 fullMapSize = { 1500, 750, 1500 };
    m_map.Load("HeightMap.png", fullMapSize, "MapTexture.png");

    // --- 1. הגדרת נ"מ (במרכז המפה) ---
    m_obstacles.clear();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> midArea(-300.0f, 300.0f);

    for (int i = 0; i < 15; i++) {
        float x = midArea(gen);
        float z = midArea(gen);
        float y = m_map.GetHeightAt(x, z);
        m_obstacles.push_back({ (Vector3){ x, y, z }, 160.0f });
    }

    m_navGraph.BuildGraphFromMap(fullMapSize, 50, m_obstacles, m_map);
    m_navGraph.PrepareGPUData();
    m_navGraph.BuildDistanceMatrix();

    // --- 2. בסיס המראה (פינה אחת: 600-, 600-) ---
    Vector3 basePos = { -600.0f, 0, -600.0f };
    basePos.y = m_map.GetHeightAt(basePos.x, basePos.z) + 2.0f;
    m_starPoint = basePos;

    // --- 3. מיקום אויב רחוק (פינה נגדית: 600+, 600+) ---
    // הגדרנו טווח רחוק בקצה החיובי של המפה
    std::uniform_real_distribution<float> farPos(500.0f, 700.0f);
    std::uniform_real_distribution<float> battleY(250.0f, 450.0f);

    Vector3 enemyStartPos = { farPos(gen), battleY(gen), farPos(gen) };

    // --- 4. המטרה (מרכז המפה - נקודת המפגש) ---
    m_targetPoint = { 0.0f, 300.0f, 0.0f };

    // --- 5. אתחול אובייקטים ---
    m_plane = std::make_unique<Plane>(m_starPoint, Vector3{0, 0, 0}, Vector3{0, 0, 0}, PINK, m_navGraph);

    // האויב מתחיל עם מהירות לכיוון המרכז (בשביל שייכנס לקרב מהר יותר)
    Vector3 enemyInitialVel = Vector3Scale(Vector3Normalize(Vector3Subtract(m_targetPoint, enemyStartPos)), NORMAL_SPEED);
    m_enemy = std::make_unique<Plane>(enemyStartPos, enemyInitialVel, Vector3{0, 0, 0}, YELLOW, m_navGraph);

    // מצלמה
    m_camera.position = { m_starPoint.x - 30, m_starPoint.y + 20, m_starPoint.z - 30 };
    m_camera.target = m_starPoint;

    m_fsm = std::make_unique<FSM>(m_navGraph);
    m_mpcController = std::make_unique<MPCController>((MPCParameters){
     40,      // Horizon
     0.1f,    // Step
     20.0f,   // Waypoint Weight (נמוך = פחות פאניקה להגיע לנקודה)
     5.0f,    // Steering Smoothness (מונע תנועות "עצבניות")
     0.2f     // Velocity Weight
 });
}

void SimsState::Draw() {
    ClearBackground(SKYBLUE);

    BeginMode3D(m_camera);

    float axisLength = 10.0f;
    float axisThick  = 0.08f;
    int slices = 8;

    DrawCylinderEx({ 0, 0, 0 }, { axisLength, 0, 0 }, axisThick, axisThick, slices, RED);
    DrawCylinderEx({ 0, 0, 0 }, { 0, axisLength, 0 }, axisThick, axisThick, slices, GREEN);
    DrawCylinderEx({ 0, 0, 0 }, { 0, 0, axisLength }, axisThick, axisThick, slices, BLUE);
    m_map.Draw();
    BeginBlendMode(BLEND_ALPHA);
    for (auto obstacle : m_obstacles) {
        DrawSphere(obstacle.pos, obstacle.radius, ColorAlpha(RED, 0.1f));
        DrawSphereWires(obstacle.pos, obstacle.radius, 10, 10, ColorAlpha(RED, 0.8f));
    }

    DrawCube(m_starPoint, 10, 2, 10, GREEN);
    DrawSphere(m_targetPoint, 1, GREEN);

    EndBlendMode();


    DrawGrid(20, 1.0f);


    m_navGraph.Draw(m_camera.position, 100);

    m_plane->Draw(m_camera);
    m_enemy->Draw(m_camera);





    EndMode3D();

    DrawText("X Axis: RED", 10, 70, 20, RED);
    DrawText("Y Axis: GREEN", 10, 95, 20, DARKGREEN);
    DrawText("Z Axis: BLUE", 10, 120, 20, BLUE);

    DrawText(TextFormat("Enemy Pos: %.2f, %.2f, %.2f",
             m_enemy->GetPosition().x, m_enemy->GetPosition().y, m_enemy->GetPosition().z),
             10, 150, 20, YELLOW);

    Vector3 pos = m_plane->GetPosition();
    float speed = m_plane->GetSpeed();

    DrawText(TextFormat("POSITION: X:%.1f Y:%.1f Z:%.1f", pos.x, pos.y, pos.z),
             10, GetScreenHeight() - 60, 20, WHITE);
    DrawText(TextFormat("AIRSPEED: %.1f KNOTS", speed),
             10, GetScreenHeight() - 35, 25, GOLD);

    DrawFPS(10, 10);
    DrawText("ENGINE MODE: ACTIVE", 10, 40, 20, DARKGREEN);
}
