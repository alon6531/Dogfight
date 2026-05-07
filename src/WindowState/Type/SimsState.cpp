//
// Created by User on 09/04/2026.
//

#include "SimsState.h"

#include "imgui.h"
#include "raymath.h"
#include "rlImGui.h"
#include "../../Engine/Engine.h"
#include <iostream>



SimsState::SimsState(Engine &engine) : State(engine), m_audioStream(), m_navGraph(), m_map(), m_starPoint(),
    m_targetPoint(),
    m_obstacleSphereModel(),
    m_obstacleWiresModel()
{

    m_camera = {0};
    m_camera.position = {10.0f, 10.0f, 10.0f};
    m_camera.target = {0.0f, 0.0f, 0.0f};
    m_camera.up = {0.0f, 1.0f, 0.0f};
    m_camera.fovy = 45.0f;
    m_camera.projection = CAMERA_PERSPECTIVE;



    DisableCursor();
    m_loadingStatus = LoadingStep::START;
}

SimsState::~SimsState() {
    UnloadAudioStream(m_audioStream);
}

void SimsState::InitializeObjects() {
    std::random_device rd;
    std::mt19937 gen(rd());


    Vector3 basePos = {-800.0f, 0, -400.0f};
    basePos.y = m_map.GetHeightAt(basePos.x, basePos.z) + 2.0f;
    m_starPoint = basePos;


    std::uniform_real_distribution<float> farPos(500.0f, 700.0f);
    std::uniform_real_distribution<float> battleY(250.0f, 450.0f);
    Vector3 enemyStartPos = {farPos(gen), battleY(gen), farPos(gen)};


    m_targetPoint = {0.0f, 0.0f, 0.0f};

    m_plane = std::make_shared<Plane>(m_starPoint, Vector3{0, 0, 0}, PINK, m_navGraph, m_targetPoint);
    m_enemy = std::make_shared<Plane>(enemyStartPos, Vector3(), YELLOW, m_navGraph, Vector3(0, 0));
    m_plane->SetEnemy(m_enemy);
    m_enemy->SetEnemy(m_plane);


    m_camera.position = {m_starPoint.x - 30, m_starPoint.y + 20, m_starPoint.z - 30};
    m_camera.target = m_starPoint;

    m_obstacleSphereModel = LoadModelFromMesh(GenMeshSphere(1.0f, 16, 16));
    m_obstacleWiresModel = LoadModelFromMesh(GenMeshSphere(1.0f, 16, 16));
    m_obstacleSphereModel.materials[0].maps[MATERIAL_MAP_ALBEDO].color = ColorAlpha(RED, 0.3f);
    m_obstacleWiresModel.materials[0].maps[MATERIAL_MAP_ALBEDO].color = ColorAlpha(BLACK, 0.5f);
    m_obstacleModelsLoaded = true;
    m_obstacles.clear();


    std::uniform_real_distribution<float> midArea(-300.0f, 300.0f);

    for (int i = 0; i < 15; i++) {
        float x = midArea(gen);
        float z = midArea(gen);
        float y = m_map.GetHeightAt(x, z);
        m_obstacles.push_back({(Vector3){x, y, z}, 160.0f});
    }
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


void SimsState::UpdateLoading() {
    switch (m_loadingStatus) {
        case LoadingStep::START:
            if (m_videoPlayer.Load("Assets/intro.mpg")) {
                m_loadingStatus = LoadingStep::VIDEO_INTRO;
            } else {
                m_loadingStatus = LoadingStep::LOADING_ASSETS;
            }
            break;

        case LoadingStep::VIDEO_INTRO:
            m_videoPlayer.Update(GetFrameTime());

            if (m_videoPlayer.IsFinished() || IsKeyPressed(KEY_SPACE)) {
                m_videoPlayer.Unload();
                m_loadingStatus = LoadingStep::LOADING_ASSETS;
            }
            break;
        case LoadingStep::LOADING_ASSETS:

        {
            Vector3 fullMapSize = {3000, 1500, 3000};
            m_map.Load("Assets/HeightMap.png", fullMapSize, "Assets/MapTexture.png");

            m_obstacleSphereModel = LoadModelFromMesh(GenMeshSphere(1.0f, 16, 16));
            m_obstacleWiresModel = LoadModelFromMesh(GenMeshSphere(1.0f, 16, 16));
            m_obstacleSphereModel.materials[0].maps[MATERIAL_MAP_ALBEDO].color = ColorAlpha(RED, 0.3f);
            m_obstacleModelsLoaded = true;
        }
            m_loadingPercentage = 70.0f;
            m_loadingStatus = LoadingStep::BUILDING_GRAPH;
            break;

        case LoadingStep::BUILDING_GRAPH:

            m_navGraph.BuildGraphFromMap({3000, 1500, 3000}, 50, m_obstacles, m_map);
            m_navGraph.PrepareGPUData();
            m_navGraph.BuildDistanceMatrix();


            InitializeObjects();

            m_loadingPercentage = 100.0f;
            m_loadingStatus = LoadingStep::READY;
            EnableCursor();
            break;
        default: ;
    }
}

void SimsState::Update(float deltaTime) {
    if (m_loadingStatus != LoadingStep::READY) {
        UpdateLoading();

    }
    else {

        deltaTime *= m_timeMultiplier;
        this->CameraHandle(deltaTime);

        m_map.UpdateFog(m_camera.position);
        m_plane->Update(deltaTime, m_navGraph, m_map, m_obstacles);
        m_enemy->Update(deltaTime, m_navGraph, m_map, m_obstacles);

        if (this->EndSimsCheck())
            p_engine.ChangeState(WindowStateType::GAME_OVER);

    }

}

void SimsState::DrawLoadingScreen() {
    ClearBackground(BLACK);
    int screenWidth = GetScreenWidth();
    int screenHeight = GetScreenHeight();


    const char* loadingText = "PREPARING BATTLEFIELD...";
    int fontSize = 30;
    int textWidth = MeasureText(loadingText, fontSize);
    DrawText(loadingText, screenWidth/2 - textWidth/2, screenHeight/2 - 60, fontSize, RAYWHITE);


    DrawRectangle(screenWidth/2 - 200, screenHeight/2 - 10, 400, 20, DARKGRAY);
    DrawRectangle(screenWidth/2 - 200, screenHeight/2 - 10, (int)(400 * (m_loadingPercentage/100.0f)), 20, LIME);

    DrawText(TextFormat("%d%%", (int)m_loadingPercentage), screenWidth/2 - 20, screenHeight/2 + 20, 20, WHITE);


    const char* debugTip = "Tip: Press TAB to toggle Debug Mode during flight";
    int tipSize = 18;
    int tipWidth = MeasureText(debugTip, tipSize);

    DrawText(debugTip, screenWidth/2 - tipWidth/2, screenHeight - 60, tipSize, GRAY);


}



void SimsState::Draw() {
    if (m_loadingStatus == LoadingStep::VIDEO_INTRO) {
        ClearBackground(BLACK);
        m_videoPlayer.Draw(0, 0, GetScreenWidth(), GetScreenHeight());
        DrawText("PRESS SPACE TO SKIP", 20, GetScreenHeight() - 40, 20, LIGHTGRAY);
        return;
    }

    if (m_loadingStatus != LoadingStep::READY) {
        DrawLoadingScreen();

    }
    else {
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




        if (p_engine.GetGameContext().m_showDebugUI)
            m_navGraph.Draw(m_camera.position, 100);

        m_plane->Draw();
        m_enemy->Draw();


        EndMode3D();



        m_plane->DrawLocked(m_camera);
        m_enemy->DrawLocked(m_camera);
        // m_enemy->DrawHub();
        rlImGuiBegin();
        m_plane->DrawHub(p_engine.GetGameContext().m_showDebugUI);
        if (p_engine.GetGameContext().m_showDebugUI) {


            // חלון בקרה כללי על המנוע (מוצב בצד שמאל למעלה)
            ImGui::SetNextWindowPos({ 10, 10 }, ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize({ 200, 100 }, ImGuiCond_FirstUseEver);

            if (ImGui::Begin("Engine Control")) {
                ImGui::Text("Time Control");
                ImGui::SliderFloat("Speed", (float*)&m_timeMultiplier, 0.0f, 10.0f, "%.1fx");

                if (ImGui::Button("Normal")) m_timeMultiplier = 1.0f;
                ImGui::SameLine();
                if (ImGui::Button("Fast")) m_timeMultiplier = 3.0f;
                ImGui::SameLine();
                if (ImGui::Button("Slow")) m_timeMultiplier = 0.2f;
            }
            ImGui::End();


        }
        rlImGuiEnd();
    }
}
