#include "FSM.h"
#include "Plane.h"
#include "raymath.h"
#include "GraphBuilder.h"

FSM::FSM(NavigationGraph& graph) : m_graph(graph) {
    InitializeTable();
}

void FSM::InitializeTable() {
    // טבלת חוקים כללית לכל מטוס
    transitionTable[{AIState::IDLE, AIEvent::OBJECTIVE_ACTIVE}] = AIState::PATROL;
    transitionTable[{AIState::PATROL, AIEvent::REACHED_OBJECTIVE}] = AIState::PURSUIT;
    transitionTable[{AIState::PURSUIT, AIEvent::ENEMY_LOST}] = AIState::IDLE;
}

void FSM::Update(Plane& actor, Plane& opponent, const Vector3& targetPos, float deltaTime) {
    AIEvent currentEvent;
    bool eventOccurred = false;


    if (actor.GetCurrentState() == AIState::IDLE) {
        currentEvent = AIEvent::OBJECTIVE_ACTIVE;
        actor.SetDestinationViaAStar(m_graph.GetClosestNode(targetPos));
        eventOccurred = true;
    }
    else if (actor.GetCurrentState() == AIState::PATROL && Vector3Distance(actor.position(), targetPos) < 5.0f) {
        currentEvent = AIEvent::REACHED_OBJECTIVE;


        opponent.SetDestinationViaAStar(m_graph.GetClosestNode(Vector3(500, -120, -500)));
        opponent.SetCurrentState(AIState::PATROL);
        opponent.SetCurrentEvent(AIEvent::OBJECTIVE_ACTIVE);
        eventOccurred = true;
    }


    if (eventOccurred) {
        AIState stateInPlane = actor.GetCurrentState();
        auto key = std::make_pair(stateInPlane, currentEvent);

        if (transitionTable.count(key)) {
            AIState nextState = transitionTable[key];
            if (nextState != stateInPlane) {
                actor.SetCurrentState(nextState);
                std::cout << "Plane [" << &actor << "] switched to: " << GetCurrentStateName(nextState) << std::endl;
            }
        }
    }
}

std::string FSM::GetCurrentStateName(AIState state) {
    switch (state) {
        case AIState::IDLE:            return "IDLE";
        case AIState::PATROL:          return "PATROL";
        case AIState::PURSUIT:         return "PURSUIT";
        case AIState::COLLISION_AVOID: return "COLLISION_AVOID";
        default:                       return "UNKNOWN";
    }
}