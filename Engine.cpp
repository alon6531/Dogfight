//
// Created by User on 02/04/2026.
//

#include "Engine.h"

#include <iostream>
#include <ostream>

#include "Global.h"
#include "raylib.h"
#include "raymath.h"
#include "State/MenuState.h"
#include "State/SimsState.h"
#include "State/GameOverState.h"

/**
 * Constructor
 */
Engine::Engine() {
    m_shouldClose = false;

    InitWindow(WIN_WIDTH, WIN_HEIGHT, "Dogfight");

    SetTargetFPS(240);





    m_state = std::make_shared<MenuState>();
    m_currentState = StateType::MENU;
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

    if (m_state) {
        StateType newState = m_state->Update(deltaTime);
        ChangeState(newState);
    }
}

void Engine::ChangeState(StateType newState) {
    // מניעת טעינה מחדש אם אנחנו כבר באותו מצב
    if (m_state && m_currentState == newState) return;
    m_currentState = newState;
    switch (newState) {
        case StateType::SIMULATION:
            m_state = std::make_shared<SimsState>();
            // אם יש פונקציית Init למצב, זה הזמן לקרוא לה
            break;

        case StateType::MENU:
            m_state = std::make_shared<MenuState>();
            break;

        case StateType::GAME_OVER:
            m_state = std::make_shared<GameOverState>();
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

