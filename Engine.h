//
// Created by User on 02/04/2026.
//

#ifndef DOGFIGHT_ENGINE_H
#define DOGFIGHT_ENGINE_H

#include <memory>

#include "State/State.h"



class Engine {
public:
    Engine();
    ~Engine();

    void Run();

private:
    void ProcessInput();

    void ChangeState(StateType newState);

    void Update(float deltaTime);
    void Render();


    StateType m_currentState;

    bool m_shouldClose = false;
    std::shared_ptr<State> m_state;


};


#endif //DOGFIGHT_ENGINE_H