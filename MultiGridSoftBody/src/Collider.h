#pragma once
#include "simpleMath.h"
struct SphereCollider {
    bool active;
    float radius;
    Point3D position;
    Point3D position_last;
    void Update();
};

struct CapsuleCollider {
    bool active;
    float radius;
    Point3D pointA;
    Point3D pointB;
    Point3D pointA_last;
    Point3D pointB_last;
    void Update();
};