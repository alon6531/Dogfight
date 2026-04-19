//
// Created by User on 09/04/2026.
//

#include "SimsState.h"

#include "raymath.h"
#include "../../Engine/Engine.h"

SimsState::SimsState(Engine &engine) : State(engine), m_map(), m_starPoint(), m_targetPoint() {
    m_camera = {0};
    m_camera.position = {10.0f, 10.0f, 10.0f};
    m_camera.target = {0.0f, 0.0f, 0.0f};
    m_camera.up = {0.0f, 1.0f, 0.0f};
    m_camera.fovy = 45.0f;
    m_camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor();

    InitializeSystem();
}

void SimsState::InitializeSystem() {
    Vector3 fullMapSize = {1500, 750, 1500};
    m_map.Load("Assets/HeightMap.png", fullMapSize, "Assets/MapTexture.png");


    if (m_obstacleModelsLoaded) {
        UnloadModel(m_obstacleSphereModel);
        UnloadModel(m_obstacleWiresModel);
    }

    m_obstacleSphereModel = LoadModelFromMesh(GenMeshSphere(1.0f, 16, 16));
    m_obstacleWiresModel = LoadModelFromMesh(GenMeshSphere(1.0f, 16, 16));
    m_obstacleSphereModel.materials[0].maps[MATERIAL_MAP_ALBEDO].color = ColorAlpha(RED, 0.3f);
    m_obstacleWiresModel.materials[0].maps[MATERIAL_MAP_ALBEDO].color = ColorAlpha(BLACK, 0.5f);
    m_obstacleModelsLoaded = true;
    m_obstacles.clear();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> midArea(-300.0f, 300.0f);

    for (int i = 0; i < 15; i++) {
        float x = midArea(gen);
        float z = midArea(gen);
        float y = m_map.GetHeightAt(x, z);
        m_obstacles.push_back({(Vector3){x, y, z}, 160.0f});
    }

    m_navGraph.BuildGraphFromMap(fullMapSize, 50, m_obstacles, m_map);
    m_navGraph.PrepareGPUData();
    m_navGraph.BuildDistanceMatrix();

    // --- 2. בסיס המראה (פינה אחת: 600-, 600-) ---
    Vector3 basePos = {-800.0f, 0, -400.0f};
    basePos.y = m_map.GetHeightAt(basePos.x, basePos.z) + 2.0f;
    m_starPoint = basePos;

    // --- 3. מיקום אויב רחוק (פינה נגדית: 600+, 600+) ---
    // הגדרנו טווח רחוק בקצה החיובי של המפה
    std::uniform_real_distribution<float> farPos(500.0f, 700.0f);
    std::uniform_real_distribution<float> battleY(250.0f, 450.0f);

    Vector3 enemyStartPos = {farPos(gen), battleY(gen), farPos(gen)};

    // --- 4. המטרה (מרכז המפה - נקודת המפגש) ---
    m_targetPoint = {0.0f, 000.0f, 0.0f};

    // --- 5. אתחול אובייקטים ---
    m_plane = std::make_shared<Plane>(m_starPoint, Vector3{0, 0, 0},PINK, m_navGraph, m_targetPoint);
    m_enemy = std::make_shared<Plane>(enemyStartPos, Vector3(), YELLOW, m_navGraph, Vector3(0, 0));
    m_plane->SetEnemy(m_enemy);
    m_enemy->SetEnemy(m_plane);

    // מצלמה
    m_camera.position = {m_starPoint.x - 30, m_starPoint.y + 20, m_starPoint.z - 30};
    m_camera.target = m_starPoint;
}

bool SimsState::EndSimsCheck() {
    bool gameOver = false;

    if (m_enemy->GetTargetLock().finalLock) {
        p_engine.GetGameContext().isVictory = false;
        gameOver = true;
    } else if (m_plane->GetTargetLock().finalLock) {
        p_engine.GetGameContext().isVictory = true;
        gameOver = true;
    }

   return gameOver;
}

void SimsState::CameraHandle(float deltaTime) {
    if (p_engine.GetGameContext().cameraMode == 1) {
        Vector3 planePos = m_plane->GetPosition();
        Vector3 planeVel = m_plane->GetVelocity();

        Vector3 forward = (Vector3LengthSqr(planeVel) > 0.1f) ? Vector3Normalize(planeVel) : Vector3{0, 0, 1};

        float distanceBehind = 60.0f;
        float heightAbove = 25.0f;

        Vector3 offset = Vector3Scale(forward, -distanceBehind);
        offset.y += heightAbove;

        Vector3 targetCameraPos = Vector3Add(planePos, offset);

        m_camera.position = Vector3Lerp(m_camera.position, targetCameraPos, 20.0f * deltaTime);
        m_camera.target = planePos;
    } else {
        int cameraSpeedMult = IsKeyDown(KEY_LEFT_SHIFT) ? 100 : 10;

        for (int i = 0; i < cameraSpeedMult; i++) UpdateCamera(&m_camera, CAMERA_FREE);
    }
}

void SimsState::Update(float deltaTime) {
    deltaTime *= 2;



    this->CameraHandle(deltaTime);


    m_map.UpdateFog(m_camera.position);
    m_plane->Update(deltaTime, m_navGraph, m_map, m_obstacles);
    m_enemy->Update(deltaTime, m_navGraph, m_map, m_obstacles);

    //std::cout << "Camera pos: " <<  m_camera.position.x << " " <<  m_camera.position.y << " " <<  m_camera.position.z << std::endl;

    if (this->EndSimsCheck())
        p_engine.ChangeState(WindowStateType::GAME_OVER);

}



void SimsState::Draw() {

    ClearBackground(SKYBLUE);

    BeginMode3D(m_camera);

    m_map.Draw();

    if (m_obstacleModelsLoaded) {
        BeginBlendMode(BLEND_ALPHA);
        for (const auto &obstacle: m_obstacles) {
            Vector3 scale = {obstacle.radius, obstacle.radius, obstacle.radius};

            DrawModelEx(m_obstacleSphereModel, obstacle.pos, {0, 1, 0}, 0.0f, scale, ColorAlpha(RED, 0.1f));
            DrawModelWiresEx(m_obstacleSphereModel, obstacle.pos, {0, 1, 0}, 0.0f, scale, ColorAlpha(RED, 0.8f));
        }
        EndBlendMode();
    }

    DrawCube(m_starPoint, 10, 2, 10, GREEN);


    EndBlendMode();





    m_navGraph.Draw(m_camera.position, 100);

    m_plane->Draw();
    m_enemy->Draw();


    EndMode3D();

    m_plane->DrawHub();
    m_plane->DrawLocked(m_camera);
    m_enemy->DrawLocked(m_camera);
    // m_enemy->DrawHub();
}
