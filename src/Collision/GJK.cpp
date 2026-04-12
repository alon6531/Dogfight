#include "GJK.h"

#include <cfloat>

#include "../Entities/Plane/Plane.h" // וודא שהנתיב נכון

Vector3 GJK::GetFarthestPointInDirection(const Plane& p, Vector3 direction) {
    // אנחנו יוצרים Bounding Box "מכוון" לפי ה-Forward וה-Up של המטוס
    Vector3 pos = p.GetPosition();
    Vector3 fwd = p.GetForward();
    Vector3 up = {0, 1, 0};
    Vector3 right = Vector3CrossProduct(fwd, up);
    up = Vector3CrossProduct(right, fwd);

    // מידות המטוס (חצי אורך/רוחב/גובה)
    float halfL = 40.0f; // אורך
    float halfW = 25.0f; // מוטת כנפיים
    float halfH = 8.0f;  // גובה

    Vector3 vertices[8];
    int i = 0;
    for (float x : {-1.0f, 1.0f})
        for (float y : {-1.0f, 1.0f})
            for (float z : {-1.0f, 1.0f})
                vertices[i++] = Vector3Add(pos, Vector3Add(Vector3Scale(fwd, x * halfL), 
                                Vector3Add(Vector3Scale(up, y * halfH), Vector3Scale(right, z * halfW))));

    float maxDot = -FLT_MAX;
    Vector3 bestPoint;
    for (const auto& v : vertices) {
        float dot = Vector3DotProduct(v, direction);
        if (dot > maxDot) {
            maxDot = dot;
            bestPoint = v;
        }
    }
    return bestPoint;
}

Vector3 GJK::Support(const Plane& a, const Plane& b, Vector3 direction) {
    return Vector3Subtract(GetFarthestPointInDirection(a, direction), 
                          GetFarthestPointInDirection(b, Vector3Negate(direction)));
}

bool GJK::CheckCollision(const Plane& a, const Plane& b) {
    Vector3 d = Vector3Subtract(b.GetPosition(), a.GetPosition());
    Simplex simplex;
    
    Vector3 s = Support(a, b, d);
    simplex.Add(s);
    d = Vector3Negate(s);

    for (int i = 0; i < 32; i++) {
        s = Support(a, b, d);
        if (Vector3DotProduct(s, d) < 0) return false;
        
        simplex.Add(s);
        if (NextSimplex(simplex, d)) return true;
    }
    return false;
}

// לוגיקת ה-Simplex (חיפוש הראשית בתוך הצורה)
bool GJK::NextSimplex(Simplex& simplex, Vector3& direction) {
    switch (simplex.size) {
        case 2: return Line(simplex, direction);
        case 3: return Triangle(simplex, direction);
        case 4: return Tetrahedron(simplex, direction);
    }
    return false;
}

bool GJK::Line(Simplex& simplex, Vector3& d) {
    Vector3 a = simplex.points[0];
    Vector3 b = simplex.points[1];
    Vector3 ab = Vector3Subtract(b, a);
    Vector3 ao = Vector3Negate(a);

    if (Vector3DotProduct(ab, ao) > 0) {
        d = Vector3CrossProduct(Vector3CrossProduct(ab, ao), ab);
    } else {
        simplex.size = 1;
        d = ao;
    }
    return false;
}

bool GJK::Triangle(Simplex& simplex, Vector3& d) {
    Vector3 a = simplex.points[0], b = simplex.points[1], c = simplex.points[2];
    Vector3 ab = Vector3Subtract(b, a), ac = Vector3Subtract(c, a), ao = Vector3Negate(a);
    Vector3 abc = Vector3CrossProduct(ab, ac);

    if (Vector3DotProduct(Vector3CrossProduct(abc, ac), ao) > 0) {
        simplex.points[1] = c; simplex.size = 2;
        d = Vector3CrossProduct(Vector3CrossProduct(ac, ao), ac);
    } else if (Vector3DotProduct(Vector3CrossProduct(ab, abc), ao) > 0) {
        simplex.size = 2;
        d = Vector3CrossProduct(Vector3CrossProduct(ab, ao), ab);
    } else {
        if (Vector3DotProduct(abc, ao) > 0) d = abc;
        else { simplex.points[1] = c; simplex.points[2] = b; d = Vector3Negate(abc); }
    }
    return false;
}

bool GJK::Tetrahedron(Simplex& simplex, Vector3& d) {
    // בודק אם הראשית בתוך הפירמידה - אם כן, יש התנגשות!
    Vector3 a = simplex.points[0], b = simplex.points[1], c = simplex.points[2], d_p = simplex.points[3];
    Vector3 ab = Vector3Subtract(b, a), ac = Vector3Subtract(c, a), ad = Vector3Subtract(d_p, a), ao = Vector3Negate(a);
    Vector3 abc = Vector3CrossProduct(ab, ac), acd = Vector3CrossProduct(ac, ad), adb = Vector3CrossProduct(ad, ab);

    if (Vector3DotProduct(abc, ao) > 0) { simplex.size = 3; return Triangle(simplex, d); }
    if (Vector3DotProduct(acd, ao) > 0) { simplex.points[1] = c; simplex.points[2] = d_p; simplex.size = 3; return Triangle(simplex, d); }
    if (Vector3DotProduct(adb, ao) > 0) { simplex.points[1] = d_p; simplex.points[2] = b; simplex.size = 3; return Triangle(simplex, d); }
    return true;
}

bool GJK::CheckCollisionAt(Vector3 simPos, Vector3 simForward, const Plane& other) {
    // אנחנו בונים "מבנה נתונים זמני" שמייצג את המטוס שלנו במיקום העתידי
    // במקום ליצור אובייקט Plane שלם, נשתמש ב-struct פשוט לצורכי GJK

    // חישוב הנקודה הכי רחוקה של המטוס שלנו במיקום החזוי
    auto GetSimSupport = [&](Vector3 direction) -> Vector3 {
        Vector3 fwd = simForward;
        Vector3 right = Vector3Normalize(Vector3CrossProduct(fwd, {0, 1, 0}));
        Vector3 up = Vector3CrossProduct(right, fwd);

        float halfL = 40.0f; float halfW = 25.0f; float halfH = 8.0f;
        Vector3 bestPoint;
        float maxDot = -FLT_MAX;

        for (float x : {-1.0f, 1.0f})
            for (float y : {-1.0f, 1.0f})
                for (float z : {-1.0f, 1.0f}) {
                    Vector3 v = Vector3Add(simPos, Vector3Add(Vector3Scale(fwd, x * halfL),
                                    Vector3Add(Vector3Scale(up, y * halfH), Vector3Scale(right, z * halfW))));
                    float dot = Vector3DotProduct(v, direction);
                    if (dot > maxDot) { maxDot = dot; bestPoint = v; }
                }
        return bestPoint;
    };

    // הרצת אלגוריתם GJK מול המטוס השני (other)
    Vector3 d = Vector3Subtract(other.GetPosition(), simPos);
    Simplex simplex;

    // Minkowski Difference Support
    auto SupportAt = [&](Vector3 dir) {
        return Vector3Subtract(GetSimSupport(dir), GetFarthestPointInDirection(other, Vector3Negate(dir)));
    };

    Vector3 s = SupportAt(d);
    simplex.Add(s);
    d = Vector3Negate(s);

    for (int i = 0; i < 32; i++) {
        s = SupportAt(d);
        if (Vector3DotProduct(s, d) < 0) return false;
        simplex.Add(s);
        if (NextSimplex(simplex, d)) return true;
    }
    return false;
}