#include "FSM.h"
#include "Plane.h"
#include "raymath.h"
#include "GraphBuilder.h"
#include <iostream>
#include "../Global.h"

#if defined(_WIN32)
    #define NOGDI
    #define NOUSER
#endif

FSM::FSM(NavigationGraph& graph) : m_graph(graph) {
    InitializeTable();
}

void FSM::InitializeTable() {
    transitionTable[{AIState::IDLE, AIEvent::OBJECTIVE_ACTIVE}] = AIState::PATROL;
    transitionTable[{AIState::PATROL, AIEvent::REACHED_OBJECTIVE}] = AIState::PURSUIT;

    transitionTable[{AIState::PATROL, AIEvent::ENEMY_SPOTTED}] = AIState::PURSUIT;
    transitionTable[{AIState::PATROL, AIEvent::DANGER_DETECTED}] = AIState::EVADE;

    transitionTable[{AIState::PURSUIT, AIEvent::DANGER_DETECTED}] = AIState::EVADE;
    transitionTable[{AIState::EVADE, AIEvent::ENEMY_SPOTTED}] = AIState::PURSUIT;

    transitionTable[{AIState::PURSUIT, AIEvent::ENEMY_LOST}] = AIState::IDLE;
}

bool FSM::UpdateLock(Plane& actor, Plane& opponent, float deltaTime) {
    const float LOCK_RANGE = 300.0f;
    const float LOCK_ANGLE = 30.0f;
    const float REQUIRED_LOCK_TIME = 3.0f;

    bool actorUnderCrosshair = IsTargetLocked(opponent, actor, LOCK_RANGE, LOCK_ANGLE);
    bool opponentUnderCrosshair = IsTargetLocked(actor, opponent, LOCK_RANGE, LOCK_ANGLE);

    actor.UpdateLockTimer(deltaTime, actorUnderCrosshair);
    opponent.UpdateLockTimer(deltaTime, opponentUnderCrosshair);

    if (actor.GetLockTimer() >= REQUIRED_LOCK_TIME) {
        actor.SetIsLocked(true);
        std::cout << "GAME OVER: Opponent locked you!" << std::endl;
        gIsVictory = false;
        return true;
    }

    if (opponent.GetLockTimer() >= REQUIRED_LOCK_TIME) {
        opponent.SetIsLocked(true);
        std::cout << "VICTORY: Enemy destroyed!" << std::endl;
        gIsVictory = true;
        return true;
    }
    return false;
}

bool FSM::Update(Plane& actor, Plane& opponent, const Vector3& targetPos, float deltaTime) {
    AIEvent currentEvent = AIEvent::NONE;
    bool eventOccurred = false;

    float myStrength = CalculateTacticalStrength(actor, opponent);
    float opponentStrength = CalculateTacticalStrength(opponent, actor);
    float distToEnemy = Vector3Distance(actor.GetPosition(), opponent.GetPosition());

    constexpr float ENGAGE_THRESHOLD = 0.6f;
    constexpr float COMBAT_DETECTION_RANGE = 450.0f;

    if (actor.GetCurrentState() == AIState::IDLE) {
        actor.SetDestinationViaAStar(m_graph.GetClosestNode(targetPos));
        currentEvent = AIEvent::OBJECTIVE_ACTIVE;
        eventOccurred = true;
    }
    else if (actor.GetCurrentState() == AIState::PATROL && Vector3Distance(actor.GetPosition(), targetPos) < 150.0f) {
        currentEvent = AIEvent::REACHED_OBJECTIVE;
        eventOccurred = true;
    }
    else if (distToEnemy < COMBAT_DETECTION_RANGE) {
        if (myStrength > opponentStrength && myStrength > ENGAGE_THRESHOLD) {
            if (actor.GetCurrentState() != AIState::PURSUIT) {
                currentEvent = AIEvent::ENEMY_SPOTTED;
                eventOccurred = true;
            }
        }
        else if (opponentStrength > myStrength && opponentStrength > ENGAGE_THRESHOLD) {
            if (actor.GetCurrentState() != AIState::EVADE) {
                currentEvent = AIEvent::DANGER_DETECTED;
                eventOccurred = true;
            }
        }
    }

    // בדיקת נעילה
    if (UpdateLock(actor, opponent, deltaTime))
        return true;


    if (eventOccurred) {
        AIState currentState = actor.GetCurrentState();
        auto key = std::make_pair(currentState, currentEvent);

        if (transitionTable.count(key)) {
            AIState nextState = transitionTable[key];

            if (nextState != currentState) {
                actor.SetCurrentState(nextState);

                // הדפסה ברורה לדיבוג
                std::cout << "\n------------------------------------" << std::endl;
                std::cout << "[FSM] Plane Address: " << (void*)&actor << std::endl;
                std::cout << "      Event Triggered: " << (int)currentEvent << std::endl;
                std::cout << "      Transition: " << GetCurrentStateName(currentState) << " -> " << GetCurrentStateName(nextState) << std::endl;
                std::cout << "------------------------------------" << std::endl;

                // עדכון האויב
                if (nextState == AIState::PURSUIT && opponent.GetCurrentState() != AIState::EVADE)
                    opponent.SetCurrentState(AIState::EVADE);
                if (nextState == AIState::EVADE && opponent.GetCurrentState() != AIState::PURSUIT)
                    opponent.SetCurrentState(AIState::PURSUIT);
            }
        }
    }

    return false;
}

bool FSM::IsTargetLocked(const Plane& attacker, const Plane& target, float maxDist, float lockAngleDeg) {
    Vector3 attackerPos = attacker.GetPosition();
    Vector3 targetPos = target.GetPosition();

    if (Vector3Distance(attackerPos, targetPos) > maxDist) return false;

    Vector3 toTarget = Vector3Normalize(Vector3Subtract(targetPos, attackerPos));
    Vector3 forward = (Vector3LengthSqr(attacker.GetVelocity()) > 0.1f)
                      ? Vector3Normalize(attacker.GetVelocity())
                      : Vector3{0, 0, 1};

    float dotProduct = Vector3DotProduct(forward, toTarget);
    float minDot = cosf((lockAngleDeg * 0.5f) * DEG2RAD);

    return dotProduct >= minDot;
}

float FSM::CalculateTacticalStrength(const Plane& owner, const Plane& target) {
    Vector3 toTarget = Vector3Normalize(Vector3Subtract(target.GetPosition(), owner.GetPosition()));
    Vector3 forward = (Vector3LengthSqr(owner.GetVelocity()) > 0.1f)
                      ? Vector3Normalize(owner.GetVelocity())
                      : Vector3{0, 0, 1};

    float facingScore = fmaxf(0.0f, Vector3DotProduct(forward, toTarget));
    float heightDiff = owner.GetPosition().y - target.GetPosition().y;
    float altitudeScore = Clamp(heightDiff / 100.0f, -1.0f, 1.0f);

    return (facingScore * 0.7f) + (altitudeScore * 0.3f);
}

std::string FSM::GetCurrentStateName(AIState state) {
    switch (state) {
        case AIState::IDLE:    return "IDLE";
        case AIState::PATROL:  return "PATROL";
        case AIState::PURSUIT: return "PURSUIT";
        case AIState::EVADE:   return "EVADE";
        default:               return "UNKNOWN";
    }
}