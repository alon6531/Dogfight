#include "MenuState.h"

MenuState::MenuState() {
    float btnWidth = 300;
    float btnHeight = 60;
    float centerX = (float)GetScreenWidth() / 2 - btnWidth / 2;

    m_firstPersonBtn = { centerX, 300, btnWidth, btnHeight };
    m_spectateBtn = { centerX, 400, btnWidth, btnHeight };
    
    m_fpColor = GRAY;
    m_specColor = GRAY;
}

StateType MenuState::Update(float deltaTime) {
    Vector2 mousePos = GetMousePosition();
    StateType nextState = StateType::MENU;


    if (CheckCollisionPointRec(mousePos, m_firstPersonBtn)) {
        m_fpColor = LIGHTGRAY;
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {

            gCameraMode = true;
            nextState = StateType::SIMULATION;
        }
    } else m_fpColor = GRAY;


    if (CheckCollisionPointRec(mousePos, m_spectateBtn)) {
        m_specColor = LIGHTGRAY;
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
           gCameraMode = false;
            nextState = StateType::SIMULATION;
        }
    } else m_specColor = GRAY;

    return nextState;
}

void MenuState::Draw() {
    ClearBackground(DARKBLUE);

    DrawText("DOGFIGHT SIMULATOR", GetScreenWidth()/2 - 200, 100, 40, WHITE);
    DrawText("Select Camera Mode to Start:", GetScreenWidth()/2 - 150, 200, 20, LIGHTGRAY);


    DrawRectangleRec(m_firstPersonBtn, m_fpColor);
    DrawText("FIRST PERSON", m_firstPersonBtn.x + 60, m_firstPersonBtn.y + 15, 20, BLACK);

    DrawRectangleRec(m_spectateBtn, m_specColor);
    DrawText("SPECTATE MODE", m_spectateBtn.x + 55, m_spectateBtn.y + 15, 20, BLACK);
}