//
// Created by User on 02/04/2026.
//

#include "Engine.h"

#include <iostream>
#include <ostream>

#include "../Global.h"
#include "raylib.h"
#include "raymath.h"
#include "../WindowState/Type/MenuState.h"
#include "../WindowState/Type/SimsState.h"
#include "../WindowState/Type/GameOverState.h"

/**
 * Constructor
 */
Engine::Engine() {
    m_shouldClose = false;

    InitWindow(WIN_WIDTH, WIN_HEIGHT, "Dogfight");

    SetTargetFPS(240);





   ChangeState(WindowStateType::MENU);
}

/**
 * Destructor
 */
Engine::~Engine() {
    CloseWindow();
}

/**
 * Incharge of all the keyboard inputs
 */
void Engine::ProcessInput() {

    if (WindowShouldClose()) m_shouldClose = true;

}




/**
 * Update every tick of the simulator
 * @param deltaTime
 */
void Engine::Update(float deltaTime) {
    deltaTime = fminf(deltaTime, 0.05f);
    if (deltaTime > 0.05f) deltaTime = 0.05f;

    if (m_state) m_state->Update(deltaTime);
}

void Engine::ChangeState(WindowStateType newState) {

    if (m_state && m_currentState == newState) return;
    m_currentState = newState;
    switch (newState) {
        case WindowStateType::SIMULATION:
            m_state = std::make_unique<SimsState>(*this);
            break;

        case WindowStateType::MENU:
            m_state = std::make_unique<MenuState>(*this);
            break;

        case WindowStateType::GAME_OVER:
            m_state = std::make_unique<GameOverState>(*this);
            break;

        default:
            std::cout << "Error: Attempted to change to an unknown state!" << std::endl;
            break;
    }
}

/**
 * Render every tick of the simulator
 */
void Engine::Render() {
    BeginDrawing();

    if (m_state) {
        m_state->Draw();
    }


    DrawFPS(WIN_WIDTH - 100, 10);

    EndDrawing();
}


void Engine::Run() {


    while (!m_shouldClose) {

        float dt = GetFrameTime();

        ProcessInput();

        Update(dt);

        Render();
    }
}

