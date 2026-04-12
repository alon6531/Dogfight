#ifndef GJK_COLLISION_H
#define GJK_COLLISION_H

#include "raylib.h"
#include "raymath.h"
#include <vector>

class GJK {
public:
    struct Simplex {
        Vector3 points[4];
        int size = 0;

        void Add(Vector3 p) {
            points[3] = points[2];
            points[2] = points[1];
            points[1] = points[0];
            points[0] = p;
            size = (size < 4) ? size + 1 : 4;
        }
    };

    // הפונקציה הראשית לבדיקת התנגשות
    static bool CheckCollision(const class Plane& a, const class Plane& b);

    static bool CheckCollisionAt(Vector3 simPos, Vector3 simForward, const Plane &other);
private:
    static Vector3 Support(const class Plane& a, const class Plane& b, Vector3 direction);
    static Vector3 GetFarthestPointInDirection(const class Plane& p, Vector3 direction);
    static bool NextSimplex(Simplex& simplex, Vector3& direction);
    
    // פונקציות עזר גיאומטריות
    static bool Line(Simplex& simplex, Vector3& direction);
    static bool Triangle(Simplex& simplex, Vector3& direction);
    static bool Tetrahedron(Simplex& simplex, Vector3& direction);


};

#endif