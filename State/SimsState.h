//
// Created by User on 09/04/2026.
//

#ifndef DOGFIGHT_SIMSSTATE_H
#define DOGFIGHT_SIMSSTATE_H
#include <memory>
#include <vector>

#include "raylib.h"
#include "../Sims/GraphBuilder.h"
#include "../Sims/Plane.h"
#include "../Sims/Map.h"
#include "../Sims/FSM.h"
#include "../Sims/MPCController.h"
#include <random>

#include "State.h"


class Plane;

class SimsState : public State {
private:
    Camera3D m_camera{};
    NavigationGraph m_navGraph;
    std::shared_ptr<Plane> m_plane;
    std::shared_ptr<Plane> m_enemy;

    std::vector<Obstacle> m_obstacles;
    Map m_map;

    Vector3 m_starPoint;
    Vector3 m_targetPoint;

    std::unique_ptr<FSM> m_fsm;
    std::unique_ptr<MPCController> m_mpcController;

public:
    explicit SimsState();

    StateType Update(float deltaTime) override;

    void InitializeSystem();

    void Draw() override;
};


#endif //DOGFIGHT_SIMSSTATE_H