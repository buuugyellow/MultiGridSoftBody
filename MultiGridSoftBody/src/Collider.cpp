#include "Collider.h"

SphereCollider::SphereCollider(Point3D pos, float radius) : m_renderObjId(-1), m_active(true), m_radius(radius), m_position(pos), m_position_last(pos) {
    vector<float> center = {m_position.x, m_position.y, m_position.z};
    CreateSphere(m_radius, 20, 20, m_vert9float, m_triIdx, center);
    m_vertNum = m_vert9float.size() / 9;
};

void SphereCollider::Update(Point3D deltaPos) {
    m_position_last = m_position;
    m_position = m_position + deltaPos;
    for (int i = 0; i < m_vertNum; i++) {
        m_vert9float[i * 9 + 0] += deltaPos.x;
        m_vert9float[i * 9 + 1] += deltaPos.y;
        m_vert9float[i * 9 + 2] += deltaPos.z;
    }
}