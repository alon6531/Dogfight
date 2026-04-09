#ifndef GAME_OVER_STATE_H
#define GAME_OVER_STATE_H

#include "State.h"
#include "raylib.h"
#include <string>

class GameOverState : public State {
public:

    explicit GameOverState();
    ~GameOverState() override = default;

    StateType Update(float deltaTime) override;
    void Draw() override;

private:

    Rectangle m_retryBtn;
    Rectangle m_menuBtn;
    
    Color m_retryColor;
    Color m_menuColor;
};

#endif