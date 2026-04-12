//
// Created by User on 09/04/2026.
//

#ifndef DOGFIGHT_SIMSSTATE_H
#define DOGFIGHT_SIMSSTATE_H
#include <memory>
#include <vector>

#include "raylib.h"
#include "../../World/GraphBuilder.h"
#include "../../Entities/Plane/Plane.h"
#include "../../World/Map.h"
#include <random>

#include "../Base/State.h"


class Plane;

class SimsState : public State {
private:
    Camera3D m_camera{};
    NavigationGraph m_navGraph;
    std::shared_ptr<Plane> m_plane;
    std::shared_ptr<Plane> m_enemy;

    std::vector<Obstacle> m_obstacles{};
    Map m_map;

    Vector3 m_starPoint;
    Vector3 m_targetPoint;

    Model m_obstacleSphereModel;
    Model m_obstacleWiresModel;
    bool m_obstacleModelsLoaded = false;

public:
    explicit SimsState(Engine& engine);

    void Update(float deltaTime) override;

    void InitializeSystem();

    void Draw() override;
};


#endif //DOGFIGHT_SIMSSTATE_H