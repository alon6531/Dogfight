//
// Created by User on 09/04/2026.
//

#ifndef DOGFIGHT_SIMSSTATE_H
#define DOGFIGHT_SIMSSTATE_H
#include <memory>
#include <vector>

#include "raylib.h"
#include "../../World/GraphBuilder.h"
#include "../../Entities/Plane/Plane.h"
#include "../../World/Map.h"
#include <random>
#include "../Base/State.h"
extern "C" {
#include "../../External/VideoPlayer/VideoPlayer.h"
}
// --- Camera initial state ---
#define CAM_START_POS_X                 10.0f
#define CAM_START_POS_Y                 10.0f
#define CAM_START_POS_Z                 10.0f
#define CAM_FOVY                        45.0f

// --- InitializeObjects ---
#define PLANE_START_POS_X              -800.0f
#define PLANE_START_POS_Z              -400.0f
#define PLANE_START_HEIGHT_OFFSET        2.0f
#define ENEMY_FAR_POS_MIN              500.0f
#define ENEMY_FAR_POS_MAX              700.0f
#define ENEMY_BATTLE_Y_MIN             250.0f
#define ENEMY_BATTLE_Y_MAX             450.0f
#define OBSTACLE_SPHERE_SEGMENTS        16
#define OBSTACLE_SPHERE_RADIUS           1.0f
#define OBSTACLE_SPHERE_ALPHA_FILL       0.3f
#define OBSTACLE_SPHERE_ALPHA_WIRE       0.8f
#define OBSTACLE_FILL_ALPHA_DRAW         0.1f
#define OBSTACLE_AREA_MIN             -300.0f
#define OBSTACLE_AREA_MAX              300.0f
#define OBSTACLE_COUNT                  15
#define OBSTACLE_RADIUS                160.0f
#define CAM_INIT_OFFSET_X              -30.0f
#define CAM_INIT_OFFSET_Y               20.0f
#define CAM_INIT_OFFSET_Z              -30.0f

// --- CameraHandle ---
#define CAM_FOLLOW_VEL_THRESHOLD         0.1f
#define CAM_FOLLOW_DISTANCE_BEHIND      60.0f
#define CAM_FOLLOW_HEIGHT_ABOVE         25.0f
#define CAM_FOLLOW_LERP_SPEED           20.0f
#define CAM_SPEED_MULT_NORMAL           10
#define CAM_SPEED_MULT_FAST            100

// --- UpdateLoading ---
#define MAP_SIZE_X                    3000.0f
#define MAP_SIZE_Y                    1500.0f
#define MAP_SIZE_Z                    3000.0f
#define MAP_HEIGHTMAP_PATH            "Assets/HeightMap.png"
#define MAP_TEXTURE_PATH              "Assets/MapTexture.png"
#define GRAPH_SPACING                   50.0f
#define LOADING_PERCENT_ASSETS          70.0f
#define LOADING_PERCENT_DONE           100.0f
#define VIDEO_INTRO_PATH              "Assets/intro.mpg"

// --- DrawLoadingScreen ---
#define LOADING_TEXT                  "PREPARING BATTLEFIELD..."
#define LOADING_TEXT_FONT_SIZE          30
#define LOADING_TEXT_Y_OFFSET          -60
#define LOADING_BAR_HALF_WIDTH         200
#define LOADING_BAR_Y_OFFSET           -10
#define LOADING_BAR_WIDTH              400
#define LOADING_BAR_HEIGHT              20
#define LOADING_PCT_FONT_SIZE           20
#define LOADING_PCT_X_OFFSET           -20
#define LOADING_PCT_Y_OFFSET            20
#define LOADING_TIP_TEXT              "Tip: Press TAB to toggle Debug Mode during flight"
#define LOADING_TIP_FONT_SIZE           18
#define LOADING_TIP_Y_FROM_BOTTOM       60

// --- Draw ---
#define SKIP_TEXT                     "PRESS SPACE TO SKIP"
#define SKIP_TEXT_FONT_SIZE             20
#define SKIP_TEXT_Y_FROM_BOTTOM         40
#define SKIP_TEXT_X                     20
#define STAR_CUBE_SIZE_X                10.0f
#define STAR_CUBE_SIZE_Y                 2.0f
#define STAR_CUBE_SIZE_Z                10.0f
#define NAV_GRAPH_RENDER_RADIUS        100.0f

// --- ImGui Debug Window ---
#define IMGUI_ENGINE_WIN_POS_X          10.0f
#define IMGUI_ENGINE_WIN_POS_Y          10.0f
#define IMGUI_ENGINE_WIN_SIZE_X        200.0f
#define IMGUI_ENGINE_WIN_SIZE_Y        100.0f
#define TIME_MULT_MIN                    0.0f
#define TIME_MULT_MAX                   10.0f
#define TIME_MULT_NORMAL                 1.0f
#define TIME_MULT_FAST                   3.0f
#define TIME_MULT_SLOW                   0.2f




enum class LoadingStep {
    START,
    VIDEO_INTRO,
    LOADING_ASSETS,
    BUILDING_GRAPH,
    READY
};

class Plane;

class SimsState : public State {
private:
    plm_t *m_plm = nullptr;
    Texture2D m_videoFrame = { 0 };
    unsigned char *m_videoRgbBuffer = nullptr;
    int m_videoWidth = 0;
    int m_videoHeight = 0;
    AudioStream m_audioStream;
    std::vector<float> m_audioQueue;

    LoadingStep m_loadingStatus = LoadingStep::START;
    float m_loadingPercentage = 0.0f;

    Camera3D m_camera{};
    NavigationGraph m_navGraph;
    std::shared_ptr<Plane> m_plane;
    std::shared_ptr<Plane> m_enemy;

    std::vector<Obstacle> m_obstacles{};
    Map m_map;

    Vector3 m_starPoint;
    Vector3 m_targetPoint;

    Model m_obstacleSphereModel;
    Model m_obstacleWiresModel;
    bool m_obstacleModelsLoaded = false;


    float m_timeMultiplier = 1.0f;
    VideoPlayer m_videoPlayer;

public:
    explicit SimsState(Engine& engine);
    ~SimsState();

    void InitializeObjects();

    bool EndSimsCheck();

    void CameraHandle(float deltaTime);



    void UpdateLoading();

    void Update(float deltaTime) override;

    void DrawLoadingScreen();


    void Draw() override;
};


#endif //DOGFIGHT_SIMSSTATE_H