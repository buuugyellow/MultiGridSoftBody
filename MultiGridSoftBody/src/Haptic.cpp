#include "Haptic.h"

#include <string>

#include "global.h"

using namespace std;

int _stdcall SetHapticState(void* pParam) {
    HapticFeeliUSB* haptic = (HapticFeeliUSB*)pParam;
    int id = haptic->m_deviceHandle;

    haptic->update_phantom(id);

    // ��ȡλ�˾��� 4*4
    haptic->get_stylus_matrix(id, &haptic->M);
    memcpy(haptic->m_buffer, haptic->M, 16 * sizeof(float));

    // ��ȡ��ǯ�Ƕ�
    haptic->ReadDeviceStates(id, haptic->State);
    haptic->GetDeviceEncoders(id, haptic->Encoders);
    haptic->GetDeviceEncoders2(id, haptic->Encoders2);
    haptic->maxValue = max(haptic->Encoders2[4], haptic->maxValue);
    if (haptic->Encoders2[4] > 0) haptic->minValue = min(haptic->Encoders2[4], haptic->minValue);
    haptic->m_buffer[16] = (haptic->Encoders2[4] - haptic->minValue) / (haptic->maxValue - haptic->minValue + 1e-8);
    haptic->m_buffer[17] = 0;

    for (int i = 0; i < 17; i++) {
        printf("%f ", haptic->m_buffer[i]);
    }
    printf("\n");

    return 0;
}

void HapticFeeliUSB::Init(string name) {
    m_name = name;

    // ���� dll
    m_driverDLL = LoadLibrary(m_dllFile);
    if (m_driverDLL == nullptr) LOG(ERROR) << m_name + " LoadLibrary ʧ��";

    createServoLoop = (int (*)())GetProcAddress(m_driverDLL, "createServoLoop");
    if (createServoLoop == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� createServoLoop";

    stopServoLoop = (int (*)())GetProcAddress(m_driverDLL, "stopServoLoop");
    if (stopServoLoop == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� stopServoLoop";

    destroyServoLoop = (int (*)())GetProcAddress(m_driverDLL, "destroyServoLoop");
    if (destroyServoLoop == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� destroyServoLoop";

    init_phantom = (int (*)(const char* configname))GetProcAddress(m_driverDLL, "init_phantom");
    if (init_phantom == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� init_phantom";

    disable_phantom = (int (*)(unsigned int index))GetProcAddress(m_driverDLL, "disable_phantom");
    if (disable_phantom == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� disable_phantom";

    startServoLoop = (int (*)(int(_stdcall * fntServo)(void*), void* lpParam)) GetProcAddress(m_driverDLL, "startServoLoop");
    if (startServoLoop == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� startServoLoop";

    update_phantom = (int (*)(unsigned int index))GetProcAddress(m_driverDLL, "update_phantom");
    if (update_phantom == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� update_phantom";

    enable_phantom_forces = (int (*)(unsigned int index))GetProcAddress(m_driverDLL, "enable_phantom_forces");
    if (enable_phantom_forces == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� enable_phantom_forces";

    disable_phantom_forces = (int (*)(unsigned int index))GetProcAddress(m_driverDLL, "disable_phantom_forces");
    if (disable_phantom_forces == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� disable_phantom_forces";

    send_phantom_force = (int (*)(unsigned int index, const float forces[3]))GetProcAddress(m_driverDLL, "send_phantom_force");
    if (send_phantom_force == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� send_phantom_force";

    get_stylus_matrix = (int (*)(unsigned int index, float(*matrix)[16]))GetProcAddress(m_driverDLL, "get_stylus_matrix");
    if (get_stylus_matrix == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� get_stylus_matrix";

    GetDeviceEncoders = (void (*)(unsigned int index, unsigned short sEncoders[9]))GetProcAddress(m_driverDLL, "GetDeviceEncoders");
    if (GetDeviceEncoders == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� GetDeviceEncoders";

    GetDeviceEncoders2 = (void (*)(unsigned int index, unsigned short sEncoders[14]))GetProcAddress(m_driverDLL, "GetDeviceEncoders2");
    if (GetDeviceEncoders2 == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� GetDeviceEncoders2";

    ReadDeviceStates = (int (*)(unsigned int index, unsigned char cState[5]))GetProcAddress(m_driverDLL, "ReadDeviceStates");
    if (ReadDeviceStates == nullptr) LOG(ERROR) << m_name + " �޷���ȡ��� ReadDeviceStates";

    int ret = createServoLoop();
    if (ret != 0) LOG(ERROR) << m_name + " createServoLoop ʧ��";

    int id = m_deviceHandle = init_phantom(m_name.c_str());
    if (id < 0) LOG(ERROR) << m_name + " init_phantom ʧ��";

    ret = enable_phantom_forces(id);
    if (ret != 0) LOG(ERROR) << m_name + " enable_phantom_forces ʧ��";

    ret = startServoLoop(SetHapticState, this);
    if (ret != 0) LOG(ERROR) << m_name + " startServoLoop ʧ��";

    LOG(INFO) << m_name + " HapticFeeliUSB::Init ����";
}

void HapticFeeliUSB::Stop() {
    int ret;
    ret = stopServoLoop();
    if (ret != 0) LOG(ERROR) << m_name + " stopServoLoop ʧ��";

    ret = destroyServoLoop();
    if (ret != 1) LOG(ERROR) << m_name + " destroyServoLoop ʧ��";

    ret = disable_phantom_forces(m_deviceHandle);
    if (ret != 0) LOG(ERROR) << m_name + " disable_phantom_forces ʧ��";

    ret = disable_phantom(m_deviceHandle);
    if (ret != 0) LOG(ERROR) << m_name + " disable_phantom ʧ��";

    LOG(INFO) << m_name + " HapticFeeliUSB::Stop ����";
}