//
// Created by User on 02/04/2026.
//

#ifndef DOGFIGHT_ENGINE_H
#define DOGFIGHT_ENGINE_H

#include <memory>

#include "../WindowState/Base/State.h"


struct GameContext {

    bool isVictory = false;
    int cameraMode = 0;


};


class Engine {
public:
    Engine();
    ~Engine();

    void Run();
    void ChangeState(WindowStateType newState);

    GameContext& GetGameContext() { return m_gameContext; };


private:
    void ProcessInput();



    void Update(float deltaTime);
    void Render();


    GameContext m_gameContext;
    WindowStateType m_currentState;

    bool m_shouldClose = false;
    std::unique_ptr<State> m_state;


};


#endif //DOGFIGHT_ENGINE_H