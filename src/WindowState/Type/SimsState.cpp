//
// Created by User on 09/04/2026.
//

#include "SimsState.h"

#include "imgui.h"
#include "raymath.h"
#include "rlImGui.h"
#include "../../Engine/Engine.h"
#include <iostream>
#define PL_MPEG_IMPLEMENTATION
#include  "../../External/pl_mpeg.h"



SimsState::SimsState(Engine &engine) : State(engine), m_map(), m_starPoint(), m_targetPoint() {
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

    // --- 2. בסיס המראה ---
    Vector3 basePos = {-800.0f, 0, -400.0f};
    basePos.y = m_map.GetHeightAt(basePos.x, basePos.z) + 2.0f;
    m_starPoint = basePos;

    // --- 3. מיקום אויב רחוק ---
    std::uniform_real_distribution<float> farPos(500.0f, 700.0f);
    std::uniform_real_distribution<float> battleY(250.0f, 450.0f);
    Vector3 enemyStartPos = {farPos(gen), battleY(gen), farPos(gen)};

    // --- 4. המטרה ---
    m_targetPoint = {0.0f, 0.0f, 0.0f};

    // --- 5. יצירת האובייקטים ---
    m_plane = std::make_shared<Plane>(m_starPoint, Vector3{0, 0, 0}, PINK, m_navGraph, m_targetPoint);
    m_enemy = std::make_shared<Plane>(enemyStartPos, Vector3(), YELLOW, m_navGraph, Vector3(0, 0));
    m_plane->SetEnemy(m_enemy);
    m_enemy->SetEnemy(m_plane);

    // הגדרת מצלמה התחלתית מעל המטוס
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



void OnVideoFrame(plm_t *player, plm_frame_t *frame, void *user) {
    if (frame == nullptr || user == nullptr) return;

    SimsState* state = (SimsState*)user;
    int width = frame->width;
    int height = frame->height;


    size_t requiredSize = (size_t)width * height * 3;

    if (state->m_videoRgbBuffer == nullptr) {
        state->m_videoRgbBuffer = (unsigned char *)malloc(requiredSize);
        state->m_videoWidth = width;
        state->m_videoHeight = height;
    }

    if (state->m_videoWidth == width && state->m_videoHeight == height) {

        plm_frame_to_rgb(frame, state->m_videoRgbBuffer, width * 3);
    }
}

void OnAudioFrame(plm_t *player, plm_samples_t *samples, void *user) {
    SimsState* state = (SimsState*)user;

    int totalSamples = samples->count * 2;


    state->m_audioQueue.insert(state->m_audioQueue.end(), samples->interleaved, samples->interleaved + totalSamples);
}

void SimsState::UpdateLoading() {
    switch (m_loadingStatus) {
        case LoadingStep::START: {
            m_plm = plm_create_with_filename("Assets/intro.mpg");


            if (m_plm != nullptr) {
                plm_set_audio_enabled(m_plm, true);

                int w = plm_get_width(m_plm);
                int h = plm_get_height(m_plm);


                Image img = { 0 };
                img.width = w;
                img.height = h;
                img.mipmaps = 1;
                img.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8;


                img.data = calloc((size_t)w * h * 3, 1);


                m_videoFrame = LoadTextureFromImage(img);

                free(img.data);

                int sampleRate = plm_get_samplerate(m_plm);
                if (sampleRate == 0) sampleRate = 48000;

                const int BUFFER_FRAMES = 4096;


                SetAudioStreamBufferSizeDefault(BUFFER_FRAMES);


                m_audioStream = LoadAudioStream(sampleRate, 32, 2);


                plm_set_audio_lead_time(m_plm, 0.5);

                PlayAudioStream(m_audioStream);

                plm_set_audio_decode_callback(m_plm, OnAudioFrame, this);
                plm_set_video_decode_callback(m_plm, OnVideoFrame, this);
                m_loadingStatus = LoadingStep::VIDEO_INTRO;
            } else {
                m_loadingStatus = LoadingStep::LOADING_ASSETS;
            }
            break;
        }

        case LoadingStep::VIDEO_INTRO:
            if (m_plm) {
                plm_decode(m_plm, GetFrameTime());

                const int chunkSize = 4096;


                while (IsAudioStreamProcessed(m_audioStream) && m_audioQueue.size() >= chunkSize * 2) {
                    // מעבירים ל-Raylib בלוק מסודר
                    UpdateAudioStream(m_audioStream, m_audioQueue.data(), chunkSize);


                    m_audioQueue.erase(m_audioQueue.begin(), m_audioQueue.begin() + (chunkSize * 2));
                }


                if (m_videoRgbBuffer != nullptr && m_videoFrame.id > 0) {
                    UpdateTexture(m_videoFrame, m_videoRgbBuffer);
                }


                if (plm_has_ended(m_plm) || IsKeyPressed(KEY_SPACE)) {

                    plm_set_video_decode_callback(m_plm, NULL, NULL);
                    StopAudioStream(m_audioStream);

                    plm_destroy(m_plm);
                    m_plm = nullptr;

                    if (m_videoRgbBuffer) {
                        free(m_videoRgbBuffer);
                        m_videoRgbBuffer = nullptr;
                    }

                    if (m_videoFrame.id > 0) {
                        UnloadTexture(m_videoFrame);
                        m_videoFrame.id = 0;
                    }

                    m_loadingStatus = LoadingStep::LOADING_ASSETS;
                }
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
    }
}

void SimsState::Update(float deltaTime) {
    if (m_loadingStatus != LoadingStep::READY) {
        UpdateLoading();
        return;
    }


    deltaTime *= m_timeMultiplier;
    this->CameraHandle(deltaTime);


    m_map.UpdateFog(m_camera.position);
    m_plane->Update(deltaTime, m_navGraph, m_map, m_obstacles);
    m_enemy->Update(deltaTime, m_navGraph, m_map, m_obstacles);

    //std::cout << "Camera pos: " <<  m_camera.position.x << " " <<  m_camera.position.y << " " <<  m_camera.position.z << std::endl;

    if (this->EndSimsCheck())
        p_engine.ChangeState(WindowStateType::GAME_OVER);

}



void SimsState::Draw() {
    if (m_loadingStatus == LoadingStep::VIDEO_INTRO) {
        ClearBackground(BLACK);
        if (m_videoFrame.id > 0) {

            DrawTexturePro(m_videoFrame,
                {0, 0, (float)m_videoFrame.width, (float)m_videoFrame.height},
                {0, 0, (float)GetScreenWidth(), (float)GetScreenHeight()},
                {0,0}, 0, WHITE);
        }
        DrawText("PRESS SPACE TO SKIP", 20, GetScreenHeight() - 40, 20, LIGHTGRAY);
    }
    else if (m_loadingStatus != LoadingStep::READY) {
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
