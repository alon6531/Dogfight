//
// Created by User on 02/04/2026.
//

#ifndef DOGFIGHT_ENGINE_H
#define DOGFIGHT_ENGINE_H

#include <memory>

#include "raylib.h"
#include "GraphBuilder.h"
#include "Plane.h"
#include "Map.h"
#include "DStarLite.h"
#include "FSM.h"
#include "MPCController.h"
#include <random>

class Engine {
public:
    Engine();
    ~Engine();

    void Run();

private:
    void ProcessInput();

    void InitializeSystem();

    void Update(float deltaTime);
    void Render();


    Camera3D m_camera{};
    bool m_shouldClose = false;
    NavigationGraph m_navGraph;
    std::shared_ptr<Plane> m_plane;
    std::shared_ptr<Plane> m_enemy;

    std::vector<Obstacle> m_obstacles;
    Map m_map;

    Vector3 m_starPoint;
    Vector3 m_targetPoint;

    std::unique_ptr<FSM> m_fsm;
    std::unique_ptr<MPCController> m_mpcController;

};


#endif //DOGFIGHT_ENGINE_H