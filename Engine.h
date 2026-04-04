//
// Created by User on 02/04/2026.
//

#ifndef DOGFIGHT_ENGINE_H
#define DOGFIGHT_ENGINE_H

#include <memory>

#include "raylib.h"
#include "GraphBuilder.h"
#include "Plane.h"


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

    Mesh m_obstacle;
    Model m_obstacleModel;
};


#endif //DOGFIGHT_ENGINE_H