//
// Created by User on 02/04/2026.
//

#ifndef DOGFIGHT_ENGINE_H
#define DOGFIGHT_ENGINE_H

#include "raylib.h"


class Engine {
public:
    Engine();
    ~Engine();

    void Run();

private:
    void ProcessInput();
    void Update(float deltaTime);
    void Render();

    Camera3D m_camera;
    bool m_shouldClose;
};


#endif //DOGFIGHT_ENGINE_H