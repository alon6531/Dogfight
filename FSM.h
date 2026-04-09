#ifndef FSM_H
#define FSM_H

#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>

#include "raylib.h"



class NavigationGraph;

enum class AIState { IDLE, PATROL, PURSUIT, EVADE, COLLISION_AVOID };
enum class AIEvent { NONE, ENEMY_SPOTTED, ENEMY_LOST, DANGER_DETECTED, OBJECTIVE_ACTIVE, REACHED_OBJECTIVE };


struct PairHash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& p) const {
        return std::hash<T1>{}(p.first) ^ (std::hash<T2>{}(p.second) << 1);
    }
};

class Plane;

class FSM {
public:
    explicit FSM(NavigationGraph& graph);
    bool Update(Plane &actor, Plane &opponent, const Vector3 &targetPos, float deltaTime);

    bool IsTargetLocked(const Plane &attacker, const Plane &target, float maxDist, float lockAngleDeg);

    static  float CalculateTacticalStrength(const Plane &owner, const Plane &target);




    static std::string GetCurrentStateName(AIState state);

private:
    NavigationGraph& m_graph;

    std::unordered_map<std::pair<AIState, AIEvent>, AIState, PairHash> transitionTable;


    void InitializeTable();


};

#endif