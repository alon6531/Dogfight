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


class Engine {
public:
    Engine();
    ~Engine();

    void Run();

private:
    void ProcessInput();
    void Update(float deltaTime);
    void Render();

    Camera3D m_camera{};
    bool m_shouldClose;
    NavigationGraph m_navGraph;
    std::unique_ptr<Plane> m_plane;

    std::vector<Obstacle> m_obstacles;
    Map m_map;

    Vector3 m_starPoint;
    Vector3 m_targetPoint;

};


#endif //DOGFIGHT_ENGINE_H