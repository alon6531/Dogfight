#include "GJK.h"
#include "../Entities/Plane/Plane.h"
#include <cfloat>


// Used to define the shape boundary for the GJK algorithm
Vector3 GJK::GetFarthestPointInDirection(Vector3 pos, Vector3 forward, Vector3 direction) {
    // We treat the plane as a simplified bounding box (OBB) for speed
    float halfWidth = 10.0f;
    float halfHeight = 5.0f;
    float halfLength = 20.0f;

    Vector3 normDir = Vector3Normalize(direction);

    // Simple ellipsoid/box approximation
    Vector3 offset = {
        normDir.x * halfLength,
        normDir.y * halfHeight,
        normDir.z * halfWidth
    };

    return Vector3Add(pos, offset);
}

// --- Support Function (Minkowski Difference) ---
Vector3 GJK::Support(Vector3 posA, Vector3 forwardA, const Plane& b, Vector3 direction) {
    // Farthest point on shape A in direction
    Vector3 p1 = GetFarthestPointInDirection(posA, forwardA, direction);
    // Farthest point on shape B in opposite direction
    Vector3 p2 = GetFarthestPointInDirection(b.GetPosition(), b.GetForward(), Vector3Negate(direction));

    return Vector3Subtract(p1, p2);
}

// --- MPC Optimized Collision Check ---
bool GJK::CheckCollisionAt(Vector3 simPos, Vector3 simForward, const Plane& other) {
    Vector3 direction = Vector3Subtract(other.GetPosition(), simPos);

    Simplex simplex;
    Vector3 support = Support(simPos, simForward, other, direction);
    simplex.Add(support);

    direction = Vector3Negate(support);

    for (int i = 0; i < 32; i++) {
        support = Support(simPos, simForward, other, direction);

        if (Vector3DotProduct(support, direction) < 0) {
            return false; // Point is past the origin, no collision
        }

        simplex.Add(support);

        if (NextSimplex(simplex, direction)) {
            return true;
        }
    }
    return false;
}

// --- Simplex Evolution Logic ---
bool GJK::NextSimplex(Simplex& simplex, Vector3& direction) {
    switch (simplex.size) {
        case 2: return Line(simplex, direction);
        case 3: return Triangle(simplex, direction);
        case 4: return Tetrahedron(simplex, direction);
    }
    return false;
}

bool GJK::Line(Simplex& simplex, Vector3& direction) {
    Vector3 a = simplex.points[0];
    Vector3 b = simplex.points[1];
    Vector3 ab = Vector3Subtract(b, a);
    Vector3 ao = Vector3Negate(a);

    if (Vector3DotProduct(ab, ao) > 0) {
        direction = Vector3CrossProduct(Vector3CrossProduct(ab, ao), ab);
    } else {
        simplex.size = 1;
        direction = ao;
    }
    return false;
}

bool GJK::Triangle(Simplex& simplex, Vector3& direction) {
    Vector3 a = simplex.points[0];
    Vector3 b = simplex.points[1];
    Vector3 c = simplex.points[2];
    Vector3 ab = Vector3Subtract(b, a);
    Vector3 ac = Vector3Subtract(c, a);
    Vector3 ao = Vector3Negate(a);
    Vector3 abc = Vector3CrossProduct(ab, ac);

    if (Vector3DotProduct(Vector3CrossProduct(abc, ac), ao) > 0) {
        if (Vector3DotProduct(ac, ao) > 0) {
            simplex.points[1] = c;
            simplex.size = 2;
            direction = Vector3CrossProduct(Vector3CrossProduct(ac, ao), ac);
        } else {
            simplex.size = 2;
            return Line(simplex, direction);
        }
    } else {
        if (Vector3DotProduct(Vector3CrossProduct(ab, abc), ao) > 0) {
            simplex.size = 2;
            return Line(simplex, direction);
        } else {
            if (Vector3DotProduct(abc, ao) > 0) {
                direction = abc;
            } else {
                simplex.points[1] = c;
                simplex.points[2] = b;
                direction = Vector3Negate(abc);
            }
        }
    }
    return false;
}

bool GJK::Tetrahedron(Simplex& simplex, Vector3& direction) {
    Vector3 a = simplex.points[0];
    Vector3 b = simplex.points[1];
    Vector3 c = simplex.points[2];
    Vector3 d = simplex.points[3];

    Vector3 ab = Vector3Subtract(b, a);
    Vector3 ac = Vector3Subtract(c, a);
    Vector3 ad = Vector3Subtract(d, a);
    Vector3 ao = Vector3Negate(a);

    Vector3 abc = Vector3CrossProduct(ab, ac);
    Vector3 acd = Vector3CrossProduct(ac, ad);
    Vector3 adb = Vector3CrossProduct(ad, ab);

    if (Vector3DotProduct(abc, ao) > 0) {
        simplex.size = 3;
        return Triangle(simplex, direction);
    }
    if (Vector3DotProduct(acd, ao) > 0) {
        simplex.points[1] = c;
        simplex.points[2] = d;
        simplex.size = 3;
        return Triangle(simplex, direction);
    }
    if (Vector3DotProduct(adb, ao) > 0) {
        simplex.points[1] = d;
        simplex.points[2] = b;
        simplex.size = 3;
        return Triangle(simplex, direction);
    }
    return true;
}

// --- Standard Collision Implementation ---
bool GJK::CheckCollision(const Plane& a, const Plane& b) {
    return CheckCollisionAt(a.GetPosition(), a.GetForward(), b);
}