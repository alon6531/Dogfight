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