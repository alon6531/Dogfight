//
// Created by User on 11/04/2026.
//

#include "PatrolState.h"

#include "raymath.h"
#include "../../Plane.h"

PatrolState::PatrolState(Plane& self, NavigationGraph& navGraph, const Vector3 &targetPos)
: AIState(self, navGraph), m_targetPos(targetPos) {

   std::vector<Vector3> tempPath = m_astar.FindPath(p_graph, p_graph.GetClosestNode(p_self.GetPosition()), p_graph.GetClosestNode(targetPos));
   for (const auto& point : tempPath) p_path.push_back(point);


}


PatrolState::~PatrolState() {
    while (!p_path.empty()) p_path.pop_front();
};

AIStateType PatrolState::Update(float deltaTime) {

    if (p_path.empty()) return AIStateType::IDLE;

    Vector3 targetPoint = p_path.front();
    float distance = Vector3Distance(p_self.GetPosition(), targetPoint);

    if (distance < 50.0f) {
        p_path.pop_front();
        if (p_path.empty()) {
            if (p_self.GetFuel() <= ESCAPE_FUEL)
                return AIStateType::FUEL;
            return AIStateType::PURSUIT;
        }

        targetPoint = p_path.front();
    }

    p_self.SteerTowards(targetPoint, deltaTime);

    //if (CanSeeEnemy()) return AIStateType::CHASE;



    return AIStateType::PATROL;
}

Vector3 PatrolState::GetCurrentTargetFromAI() {
    return m_targetPos;
}
