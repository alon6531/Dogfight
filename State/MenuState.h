#ifndef MENU_STATE_H
#define MENU_STATE_H

#include "State.h"
#include "raylib.h"

class MenuState : public State {
public:
    MenuState();
    ~MenuState() override = default;

    StateType Update(float deltaTime) override;
    void Draw() override;

private:

    Rectangle m_firstPersonBtn;
    Rectangle m_spectateBtn;
    

    Color m_fpColor;
    Color m_specColor;

    bool IsPointInRect(Vector2 point, Rectangle rect);
};

#endif