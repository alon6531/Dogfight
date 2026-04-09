#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

#include "raylib.h"
#include <vector>

struct MPCParameters {
    int horizon = 10;
    float dt = 0.1f;
    float maxForce = 50.0f;
    float weightPos = 1.0f;
    float weightSmooth = 0.5f;
};

class MPCController {
public:
    MPCController(MPCParameters params) : m_params(params) {}


    Vector3 CalculateSteering(Vector3 pos, Vector3 vel, const std::vector<Vector3>& path) const;
    static Vector3 Vector3RotateTowards(Vector3 current, Vector3 target, float maxAngle);


private:

    MPCParameters m_params;
};

#endif