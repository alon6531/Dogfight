#include "GameOverState.h"

#include "../../Engine/Engine.h"

GameOverState::GameOverState(Engine& engine) : State(engine) {
    EnableCursor();


    float btnWidth = 300;
    float btnHeight = 60;
    float centerX = (float)GetScreenWidth() / 2 - btnWidth / 2;

    m_retryBtn = { centerX, 350, btnWidth, btnHeight };
    m_menuBtn = { centerX, 450, btnWidth, btnHeight };
    
    m_retryColor = GRAY;
    m_menuColor = GRAY;
}

void GameOverState::Update(float deltaTime) {
    Vector2 mousePos = GetMousePosition();
    

    if (CheckCollisionPointRec(mousePos, m_retryBtn)) {
        m_retryColor = LIGHTGRAY;
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) p_engine.ChangeState(WindowStateType::SIMULATION);
    } else m_retryColor = GRAY;


    if (CheckCollisionPointRec(mousePos, m_menuBtn)) {
        m_menuColor = LIGHTGRAY;
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) return p_engine.ChangeState(WindowStateType::MENU);
    } else m_menuColor = GRAY;

}

void GameOverState::Draw() {

    ClearBackground(BLACK);

    std::string title = p_engine.GetGameContext().isVictory ? "VICTORY!" : "MISSION FAILED";
    Color titleColor =  p_engine.GetGameContext().isVictory ? GOLD : RED;

    DrawText(title.c_str(), GetScreenWidth()/2 - MeasureText(title.c_str(), 50)/2, 150, 50, titleColor);


    DrawRectangleRec(m_retryBtn, m_retryColor);
    DrawText("RETRY MISSION", m_retryBtn.x + 65, m_retryBtn.y + 20, 20, BLACK);


    DrawRectangleRec(m_menuBtn, m_menuColor);
    DrawText("MAIN MENU", m_menuBtn.x + 95, m_menuBtn.y + 20, 20, BLACK);
}