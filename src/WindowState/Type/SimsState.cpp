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
    m_camera.position   = { CAM_START_POS_X, CAM_START_POS_Y, CAM_START_POS_Z };
    m_camera.target     = { 0.0f, 0.0f, 0.0f };
    m_camera.up         = { 0.0f, 1.0f, 0.0f };
    m_camera.fovy       = CAM_FOVY;
    m_camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor();
    m_loadingStatus = LoadingStep::START;
}

SimsState::~SimsState() {


}

void SimsState::InitializeObjects() {
    std::random_device rd;
    std::mt19937 gen(rd());

    Vector3 basePos = { PLANE_START_POS_X, 0, PLANE_START_POS_Z };
    basePos.y = m_map.GetHeightAt(basePos.x, basePos.z) + PLANE_START_HEIGHT_OFFSET;
    m_starPoint = basePos;

    std::uniform_real_distribution<float> farPos(ENEMY_FAR_POS_MIN, ENEMY_FAR_POS_MAX);
    std::uniform_real_distribution<float> battleY(ENEMY_BATTLE_Y_MIN, ENEMY_BATTLE_Y_MAX);
    Vector3 enemyStartPos = { farPos(gen), battleY(gen), farPos(gen) };

    m_targetPoint = { 0.0f, 0.0f, 0.0f };

    m_plane = std::make_shared<Plane>(m_starPoint, Vector3{0, 0, 0}, PINK, m_navGraph, m_targetPoint);
    m_enemy = std::make_shared<Plane>(enemyStartPos, Vector3(), YELLOW, m_navGraph, Vector3(0, 0));
    m_plane->SetEnemy(m_enemy);
    m_enemy->SetEnemy(m_plane);

    m_camera.position = {
        m_starPoint.x + CAM_INIT_OFFSET_X,
        m_starPoint.y + CAM_INIT_OFFSET_Y,
        m_starPoint.z + CAM_INIT_OFFSET_Z
    };
    m_camera.target = m_starPoint;

    m_obstacleSphereModel = LoadModelFromMesh(GenMeshSphere(OBSTACLE_SPHERE_RADIUS, OBSTACLE_SPHERE_SEGMENTS, OBSTACLE_SPHERE_SEGMENTS));
    m_obstacleWiresModel  = LoadModelFromMesh(GenMeshSphere(OBSTACLE_SPHERE_RADIUS, OBSTACLE_SPHERE_SEGMENTS, OBSTACLE_SPHERE_SEGMENTS));
    m_obstacleSphereModel.materials[0].maps[MATERIAL_MAP_ALBEDO].color = ColorAlpha(RED, OBSTACLE_SPHERE_ALPHA_FILL);
    m_obstacleWiresModel.materials[0].maps[MATERIAL_MAP_ALBEDO].color  = ColorAlpha(BLACK, OBSTACLE_SPHERE_ALPHA_WIRE);
    m_obstacleModelsLoaded = true;
    m_obstacles.clear();

    std::uniform_real_distribution<float> midArea(OBSTACLE_AREA_MIN, OBSTACLE_AREA_MAX);
    for (int i = 0; i < OBSTACLE_COUNT; i++) {
        float x = midArea(gen);
        float z = midArea(gen);
        float y = m_map.GetHeightAt(x, z);
        m_obstacles.push_back({ (Vector3){x, y, z}, OBSTACLE_RADIUS });
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

        Vector3 forward = (Vector3LengthSqr(planeVel) > CAM_FOLLOW_VEL_THRESHOLD)
                          ? Vector3Normalize(planeVel)
                          : Vector3{0, 0, 1};

        Vector3 offset = Vector3Scale(forward, -CAM_FOLLOW_DISTANCE_BEHIND);
        offset.y += CAM_FOLLOW_HEIGHT_ABOVE;

        Vector3 targetCameraPos = Vector3Add(planePos, offset);

        m_camera.position = Vector3Lerp(m_camera.position, targetCameraPos, CAM_FOLLOW_LERP_SPEED * deltaTime);
        m_camera.target   = planePos;
    } else {
        int cameraSpeedMult = IsKeyDown(KEY_LEFT_SHIFT) ? CAM_SPEED_MULT_FAST : CAM_SPEED_MULT_NORMAL;
        for (int i = 0; i < cameraSpeedMult; i++) UpdateCamera(&m_camera, CAMERA_FREE);
    }
}

void SimsState::UpdateLoading() {
    switch (m_loadingStatus) {
        case LoadingStep::START:
            if (m_videoPlayer.Load(VIDEO_INTRO_PATH)) {
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
            Vector3 fullMapSize = { MAP_SIZE_X, MAP_SIZE_Y, MAP_SIZE_Z };
            m_map.Load(MAP_HEIGHTMAP_PATH, fullMapSize, MAP_TEXTURE_PATH);

            m_obstacleSphereModel = LoadModelFromMesh(GenMeshSphere(OBSTACLE_SPHERE_RADIUS, OBSTACLE_SPHERE_SEGMENTS, OBSTACLE_SPHERE_SEGMENTS));
            m_obstacleWiresModel  = LoadModelFromMesh(GenMeshSphere(OBSTACLE_SPHERE_RADIUS, OBSTACLE_SPHERE_SEGMENTS, OBSTACLE_SPHERE_SEGMENTS));
            m_obstacleSphereModel.materials[0].maps[MATERIAL_MAP_ALBEDO].color = ColorAlpha(RED, OBSTACLE_SPHERE_ALPHA_FILL);
            m_obstacleModelsLoaded = true;
        }
            m_loadingPercentage = LOADING_PERCENT_ASSETS;
            m_loadingStatus     = LoadingStep::BUILDING_GRAPH;
            break;

        case LoadingStep::BUILDING_GRAPH:
            m_navGraph.BuildGraphFromMap({ MAP_SIZE_X, MAP_SIZE_Y, MAP_SIZE_Z }, GRAPH_SPACING, m_obstacles, m_map);
            m_navGraph.PrepareGPUData();
            m_navGraph.BuildDistanceMatrix();

            InitializeObjects();

            m_loadingPercentage = LOADING_PERCENT_DONE;
            m_loadingStatus     = LoadingStep::READY;
            EnableCursor();
            break;

        default: ;
    }
}

void SimsState::Update(float deltaTime) {
    if (m_loadingStatus != LoadingStep::READY) {
        UpdateLoading();
    } else {
        if (this->EndSimsCheck())
            p_engine.ChangeState(WindowStateType::GAME_OVER);
        else
            {
            deltaTime *= m_timeMultiplier;
            this->CameraHandle(deltaTime);

            m_map.UpdateFog(m_camera.position);
            m_plane->Update(deltaTime, m_navGraph, m_map, m_obstacles);
            m_enemy->Update(deltaTime, m_navGraph, m_map, m_obstacles);
        }


    }
}

void SimsState::DrawLoadingScreen() {
    ClearBackground(BLACK);
    int screenWidth  = GetScreenWidth();
    int screenHeight = GetScreenHeight();

    int textWidth = MeasureText(LOADING_TEXT, LOADING_TEXT_FONT_SIZE);
    DrawText(LOADING_TEXT,
             screenWidth / 2 - textWidth / 2,
             screenHeight / 2 + LOADING_TEXT_Y_OFFSET,
             LOADING_TEXT_FONT_SIZE, RAYWHITE);

    DrawRectangle(screenWidth / 2 - LOADING_BAR_HALF_WIDTH,
                  screenHeight / 2 + LOADING_BAR_Y_OFFSET,
                  LOADING_BAR_WIDTH, LOADING_BAR_HEIGHT, DARKGRAY);
    DrawRectangle(screenWidth / 2 - LOADING_BAR_HALF_WIDTH,
                  screenHeight / 2 + LOADING_BAR_Y_OFFSET,
                  (int)(LOADING_BAR_WIDTH * (m_loadingPercentage / LOADING_PERCENT_DONE)),
                  LOADING_BAR_HEIGHT, LIME);

    DrawText(TextFormat("%d%%", (int)m_loadingPercentage),
             screenWidth / 2 + LOADING_PCT_X_OFFSET,
             screenHeight / 2 + LOADING_PCT_Y_OFFSET,
             LOADING_PCT_FONT_SIZE, WHITE);

    int tipWidth = MeasureText(LOADING_TIP_TEXT, LOADING_TIP_FONT_SIZE);
    DrawText(LOADING_TIP_TEXT,
             screenWidth / 2 - tipWidth / 2,
             screenHeight - LOADING_TIP_Y_FROM_BOTTOM,
             LOADING_TIP_FONT_SIZE, GRAY);
}

void SimsState::Draw() {
    if (m_loadingStatus == LoadingStep::VIDEO_INTRO) {
        ClearBackground(BLACK);
        m_videoPlayer.Draw(0, 0, GetScreenWidth(), GetScreenHeight());
        DrawText(SKIP_TEXT, SKIP_TEXT_X, GetScreenHeight() - SKIP_TEXT_Y_FROM_BOTTOM, SKIP_TEXT_FONT_SIZE, LIGHTGRAY);
        return;
    }

    if (m_loadingStatus != LoadingStep::READY) {
        DrawLoadingScreen();
    } else {
        ClearBackground(SKYBLUE);

        BeginMode3D(m_camera);

        m_map.Draw();

        if (m_obstacleModelsLoaded) {
            BeginBlendMode(BLEND_ALPHA);
            for (const auto &obstacle : m_obstacles) {
                Vector3 scale = { obstacle.radius, obstacle.radius, obstacle.radius };
                DrawModelEx(m_obstacleSphereModel, obstacle.pos, {0, 1, 0}, 0.0f, scale, ColorAlpha(RED, OBSTACLE_FILL_ALPHA_DRAW));
                DrawModelWiresEx(m_obstacleSphereModel, obstacle.pos, {0, 1, 0}, 0.0f, scale, ColorAlpha(RED, OBSTACLE_SPHERE_ALPHA_WIRE));
            }
            EndBlendMode();
        }

        DrawCube(m_starPoint, STAR_CUBE_SIZE_X, STAR_CUBE_SIZE_Y, STAR_CUBE_SIZE_Z, GREEN);

        if (p_engine.GetGameContext().m_showDebugUI)
            m_navGraph.Draw(m_camera.position, NAV_GRAPH_RENDER_RADIUS);

        m_plane->Draw();
        m_enemy->Draw();

        EndMode3D();

        m_plane->DrawLocked(m_camera);
        m_enemy->DrawLocked(m_camera);

        rlImGuiBegin();
        m_plane->DrawHub(p_engine.GetGameContext().m_showDebugUI);

        if (p_engine.GetGameContext().m_showDebugUI) {
            ImGui::SetNextWindowPos( { IMGUI_ENGINE_WIN_POS_X,  IMGUI_ENGINE_WIN_POS_Y  }, ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize({ IMGUI_ENGINE_WIN_SIZE_X, IMGUI_ENGINE_WIN_SIZE_Y }, ImGuiCond_FirstUseEver);

            if (ImGui::Begin("Engine Control")) {
                ImGui::Text("Time Control");
                ImGui::SliderFloat("Speed", (float*)&m_timeMultiplier, TIME_MULT_MIN, TIME_MULT_MAX, "%.1fx");

                if (ImGui::Button("Normal")) m_timeMultiplier = TIME_MULT_NORMAL;
                ImGui::SameLine();
                if (ImGui::Button("Fast"))   m_timeMultiplier = TIME_MULT_FAST;
                ImGui::SameLine();
                if (ImGui::Button("Slow"))   m_timeMultiplier = TIME_MULT_SLOW;
            }
            ImGui::End();
        }
        rlImGuiEnd();
    }
}