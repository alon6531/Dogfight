//
// Created by User on 09/04/2026.
//

#ifndef DOGFIGHT_STATE_H
#define DOGFIGHT_STATE_H

#include "../../Global.h"

class Engine;

enum class WindowStateType {
    MENU,
    SIMULATION,
    PAUSE,
    GAME_OVER,
    VICTORY
};

class State {
protected:
    Engine& p_engine;


public:
    explicit State(Engine& engine) : p_engine(engine) {};
    virtual ~State() = default;

    virtual void Update(float deltaTime) = 0;
    virtual void Draw() = 0;


};


#endif //DOGFIGHT_STATE_H