#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

#include <memory>

#include "raylib.h"
#include <vector>


// --- CalculateBestSteer ---
#define MPC_NUM_SAMPLES                 15
#define MPC_JITTER_RANGE               100
#define MPC_JITTER_SCALE               100.0f
#define MPC_JITTER_BLEND                 0.5f

// --- SimulateAndGetCost ---
#define MPC_DISTANCE_WEIGHT              1.0f
#define MPC_COLLISION_PENALTY        10000.0f
#define MPC_GROUND_MIN_HEIGHT           20.0f
#define MPC_GROUND_PENALTY             500.0f

class Plane;

struct MPCResult {
    Vector3 bestSteer;
    float cost;
};

class MPCController {
public:
    MPCController(int horizon = 10, float dt = 0.1f)
        : m_horizon(horizon), m_predictionDt(dt) {}


    Vector3 CalculateBestSteer(Vector3 pos, Vector3 vel, Vector3 forward, Vector3 target, const std::vector<struct Obstacle> &obstacles, const std::shared_ptr<Plane> &enem);

    float SimulateAndGetCost(Vector3 pos, Vector3 vel, Vector3 steerDir, Vector3 target,
                             const std::vector<Obstacle> &obstacles, const std::shared_ptr<Plane> &enemy) const;

private:
    int m_horizon;
    float m_predictionDt;


};


#endif