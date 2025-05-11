#pragma once
#include <string>
#include "simpleMath.h"
struct SphereCollider {
    int m_renderObjId;
    bool m_active;
    float m_radius;
    Point3D m_position;
    Point3D m_position_last;

    int m_vertNum;
    //vector<float> m_vertPos;
    vector<float> m_vert9float;
    vector<unsigned int> m_triIdx;
    
    SphereCollider(Point3D pos, float radius);
    void Update(Point3D deltaPos);
};

struct CapsuleCollider {
    bool m_active;
    float m_radius;
    Point3D m_pointA;
    Point3D m_pointB;
    Point3D m_pointA_last;
    Point3D m_pointB_last;
    void Update();
};