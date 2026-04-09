//
// Created by User on 09/04/2026.
//

#ifndef DOGFIGHT_STATE_H
#define DOGFIGHT_STATE_H

#include "../Global.h"

enum class StateType {
    MENU,
    SIMULATION,
    PAUSE,
    GAME_OVER,
    VICTORY
};

class State {
public:
    explicit State() = default;
    virtual ~State() = default;

    virtual StateType Update(float deltaTime) = 0;
    virtual void Draw() = 0;


};


#endif //DOGFIGHT_STATE_H