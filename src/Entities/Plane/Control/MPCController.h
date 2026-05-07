#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

#include <memory>

#include "raylib.h"
#include <vector>



#include "raylib.h"
#include <vector>

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