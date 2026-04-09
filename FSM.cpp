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
    transitionTable[{AIState::PURSUIT, AIEvent::DANGER_DETECTED}] = AIState::EVADE;
    transitionTable[{AIState::EVADE, AIEvent::ENEMY_SPOTTED}] = AIState::PURSUIT;
}

void FSM::Update(Plane& actor, Plane& opponent, const Vector3& targetPos, float deltaTime) {
    AIEvent currentEvent;
    bool eventOccurred = false;


    if (actor.GetCurrentState() == AIState::IDLE) {
        currentEvent = AIEvent::OBJECTIVE_ACTIVE;
        actor.SetDestinationViaAStar(m_graph.GetClosestNode(targetPos));
        eventOccurred = true;
    }
    else if (actor.GetCurrentState() == AIState::PATROL && Vector3Distance(actor.position(), targetPos) < 40.0f) {
        currentEvent = AIEvent::REACHED_OBJECTIVE;


        opponent.SetCurrentState(AIState::EVADE);
        opponent.SetCurrentEvent(AIEvent::DANGER_DETECTED);
        eventOccurred = true;
    }
    if (IsEnemyBehind(actor, opponent.position())) {
        currentEvent = AIEvent::DANGER_DETECTED;
        eventOccurred = true;

        opponent.SetCurrentState(AIState::PURSUIT);
        opponent.SetCurrentEvent(AIEvent::ENEMY_SPOTTED);
    }
    if (IsEnemyBehind(opponent, actor.position())) {
        currentEvent = AIEvent::ENEMY_SPOTTED;
        eventOccurred = true;

        opponent.SetCurrentState(AIState::EVADE);
        opponent.SetCurrentEvent(AIEvent::DANGER_DETECTED);
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

bool FSM::IsEnemyBehind(const Plane& owner, const Vector3& enemyPos) {
    Vector3 toEnemy = Vector3Normalize(Vector3Subtract(enemyPos, owner.position()));
    Vector3 forward = Vector3Normalize(owner.velocity());


    return Vector3DotProduct(forward, toEnemy) < -0.5f;
}

std::string FSM::GetCurrentStateName(AIState state) {
    switch (state) {
        case AIState::IDLE:            return "IDLE";
        case AIState::PATROL:          return "PATROL";
        case AIState::PURSUIT:         return "PURSUIT";
        case AIState::COLLISION_AVOID: return "COLLISION_AVOID";
        case AIState::EVADE:           return "EVADE";
        default:                       return "UNKNOWN";
    }
}