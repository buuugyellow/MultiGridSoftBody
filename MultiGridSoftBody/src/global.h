#include <string>

#include "glog/logging.h"
using namespace std;
extern string config_objName;
extern string config_dataDir;
extern vector<float> g_pointsForRender;  // �����߳���Ⱦ�̹߳���Ķ������ݣ���Ҫ���ݵİ������������� + ������ + UV����
extern vector<float> g_normalsForRender;
extern vector<float> g_uvForRender;

void OutputPosNormIndex(string filepath, std::vector<float> pos, std::vector<float> norm, std::vector<unsigned int> index);