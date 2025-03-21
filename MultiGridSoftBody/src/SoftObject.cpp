#include "SoftObject.h"
#include <fstream>
#include <set>
using namespace std;
void SoftObject::ReadFromFile() {
    const uint32_t kMaxLineLength = 1024;
    char buffer[kMaxLineLength];

#pragma region
    fstream file(m_objFile.c_str());
    if (!file) return;

    float x, y, z, u, v;
    while (file) {
        file >> buffer;

        if (strcmp(buffer, "vn") == 0) {  // normals
            file >> x >> y >> z;
        } else if (strcmp(buffer, "vt") == 0) {
            // texture coords
            file >> u >> v;
            m_triUV.push_back(u);
            m_triUV.push_back(v);
        } else if (buffer[0] == 'v') {  // positions
            file >> x >> y >> z;
            m_triVertPos.push_back(x);
            m_triVertPos.push_back(y);
            m_triVertPos.push_back(z);
        } else if (buffer[0] == 'f') {
            // faces
            int pi[3];   // 三角形三个点
            int uvi[3];  // 三角形三个点的uv坐标
            for (int i = 0; i < 3; ++i) {
                int v = -1;
                int vt = 1;
                int vn = -1;

                file >> v;
                if (!file.eof()) {
                    // failed to read another index continue on
                    if (file.fail()) {
                        file.clear();
                        break;
                    }

                    if (file.peek() == '/') {
                        file.ignore();

                        if (file.peek() != '/') {
                            file >> vt;
                        }

                        if (file.peek() == '/') {
                            file.ignore();
                            file >> vn;
                        }
                    }
                }

                pi[i] = v - 1;
                uvi[i] = vt - 1;
            }  // for (int i = 0; i < 3; ++i)
            m_triIdx.push_back(pi[0]);
            m_triIdx.push_back(pi[1]);
            m_triIdx.push_back(pi[2]);
            m_triUVIdx.push_back(uvi[0]);
            m_triUVIdx.push_back(uvi[1]);
            m_triUVIdx.push_back(uvi[2]);
        }  // else if (buffer[0] == 'f')
        else {
            char linebuf[1024];
            file.getline(linebuf, 1024);
        }
    }
    file.close();

    if (0 == m_triUV.size()) m_triUV.resize(m_triVertPos.size() / 3 * 2);
#pragma endregion 读取OBJ文件

#pragma region
    LOG(INFO) << ("开始读取四面体：" + m_tetFile);
    
    int eleNum;
    int number = 0;

    file.open(m_tetFile);
    if (!file) {
        LOG(ERROR) << ("打开四面体文件错误：" + m_tetFile);
        return;
    }
    while (file) {
        file >> buffer;
        if (strcmp(buffer, "$Nodes") == 0) {
            file >> number;
            unsigned int idx;
            float x, y, z;
            LOG(INFO) << ("四面体顶点数量：" + to_string(number));
            for (int i = 0; i < number; i++) {
                file >> idx >> x >> y >> z;
                m_tetVertPosORIG.push_back(x);
                m_tetVertPosORIG.push_back(y);
                m_tetVertPosORIG.push_back(z);
            }
        } else if (strcmp(buffer, "$Elements") == 0) {
            file >> eleNum;
            int idx, type, tag, phy, elem;
            unsigned int i0, i1, i2, i3;
            LOG(INFO) << ("四面体数量：" + to_string(eleNum));
            for (int i = 0; i < eleNum; i++) {
                file >> idx >> type >> tag >> phy >> elem;
                if (type == 2) {  // surface indices
                    file >> i0 >> i1 >> i2;
                } else if (type == 4) {  // tet indices
                    file >> i0 >> i1 >> i2 >> i3;
                    m_tetIdxORIG.push_back(i0 - 1);
                    m_tetIdxORIG.push_back(i1 - 1);
                    m_tetIdxORIG.push_back(i2 - 1);
                    m_tetIdxORIG.push_back(i3 - 1);
                }
            }
        }
    }
    file.close();
    LOG(INFO) << ("读取完毕：" + m_tetFile);
#pragma endregion 读取四面体文件

    
#ifdef _DEBUG
    // 检测是否有重合顶点
    size_t tetVertNumO = m_tetVertPosORIG.size() / 3;
    for (size_t i = 0; i < tetVertNumO - 1; i++) {
        float trix = m_tetVertPosORIG[i * 3];
        float triy = m_tetVertPosORIG[i * 3 + 1];
        float triz = m_tetVertPosORIG[i * 3 + 2];
        for (size_t j = i + 1; j < tetVertNumO; j++) {
            float tetx = m_tetVertPosORIG[j * 3];
            float tety = m_tetVertPosORIG[j * 3 + 1];
            float tetz = m_tetVertPosORIG[j * 3 + 2];
            float dist = sqrt((trix - tetx) * (trix - tetx) + (triy - tety) * (triy - tety) + (triz - tetz) * (triz - tetz));
            if (dist < 1e-5f) {
                LOG(ERROR) << (m_name + " 两个顶点重合：" + to_string(i) + "-" + to_string(j));
            }
        }
    }
#endif  // _DEBUG

    
#pragma endregion 检测重合四面体顶点
}