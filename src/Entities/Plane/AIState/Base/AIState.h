//
// Created by User on 11/04/2026.
//

#ifndef DOGFIGHT_AISTATE_H
#define DOGFIGHT_AISTATE_H

#include "../../../../Navigation/DStarLite.h"

enum class AIStateType {
    IDLE,
    PATROL,
    PURSUIT,
    EVASION, TAKEOFF, FUEL
};

class Plane;

class AIState {
protected:
    Plane& p_self;
    NavigationGraph& p_graph;
    std::deque<Vector3> p_path;

public:
    AIState(Plane& self, NavigationGraph& navGraph) : p_self(self), p_graph(navGraph) {}
    virtual ~AIState() = default;

    virtual AIStateType Update(float deltaTime) = 0;

    const std::deque<Vector3>& GetPath() { return p_path; }

    virtual Vector3 GetCurrentTargetFromAI() = 0;

};


#endif //DOGFIGHT_AISTATE_H