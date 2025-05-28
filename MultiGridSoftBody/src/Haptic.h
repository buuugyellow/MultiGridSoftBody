#pragma once
#include <string>
#include <Windows.h>
using namespace std;
#define TransSize 64

struct Haptic {
    virtual void Init(string name) = 0;
    virtual void Stop() = 0;
};

struct HapticFeeliUSB : Haptic {
    const wchar_t* m_dllFile = L"FeeliUSB.dll";
    HMODULE m_driverDLL;
    string m_name;
    int m_deviceHandle = -1;

    float M[16]; // �豸�ʼ���Ƶ�λ�ˣ���Σ�����
    unsigned char State[5];
    unsigned short Encoders[16];
    unsigned short Encoders2[16];
    unsigned short maxValue = 0;
    unsigned short minValue = INT16_MAX;
    float m_buffer[TransSize]; // ����λ�˾���ͼ�ǯ�Ƕ�

    void Init(string name);
    void Stop();

    /// <summary>
    /// ��������ѭ��
    /// </summary>
    /// <returns>0: �ɹ� -1: ʧ��</returns>
    int (*createServoLoop)();

    /// <summary>
    /// ֹͣ����ѭ��
    /// </summary>
    /// <returns>0: �ɹ� -1: ʧ��</returns>
    int (*stopServoLoop)();

    /// <summary>
    /// ��ֹ����ѭ��
    /// </summary>
    /// <returns>0: ʧ�� 1:�ɹ�</returns>
    int (*destroyServoLoop)();

    /// <summary>
    /// ��ʼ���豸
    /// </summary>
    /// <param name="configname">�豸���ƣ��������ļ�ͬ����</param>
    /// <returns>-10: ʧ�ܣ�>=0: �豸������</returns>
    int (*init_phantom)(const char* configname);

    /// <summary>
    /// ʹ���豸
    /// </summary>
    /// <param name="index">�豸���</param>
    /// <returns>-11:ʧ�� 0:�ɹ�</returns>
    int (*disable_phantom)(unsigned int index);

    /// <summary>
    /// ��������ѭ��
    /// </summary>
    /// <param name="func">�ص�����ָ��</param>
    /// <param name="lpParam">�ص���������ָ��</param>
    /// <returns>-11: �ص�����ָ��ΪNULL -1:ʧ�ܻ����ѭ�������� 0:�����ɹ�</returns>
    int (*startServoLoop)(int(_stdcall* fntServo)(void*), void* lpParam);

    /// <summary>
    /// ȡ�豸�ʼ���Ƶ�λ�ˣ���Σ�����
    /// </summary>
    /// <param name="index">�豸���</param>
    /// <param name="matrix">λ�ˣ���Σ�����</param>
    /// <returns>-11:ʧ�� 0:�ɹ�</returns>
    int (*get_stylus_matrix)(unsigned int index, float (*matrix)[16]);

    /// <summary>
    /// �����豸
    /// </summary>
    /// <param name="index">�豸���</param>
    /// <returns>-11:ʧ�� 0:�ɹ� �������豸״ֵ̬</returns>
    int (*update_phantom)(unsigned int index);

    /// <summary>
    /// �豸������ʹ��
    /// </summary>
    /// <param name="index">�豸���</param>
    /// <returns>-11:ʧ�� 0:�ɹ�</returns>
    int (*enable_phantom_forces)(unsigned int index);

    /// <summary>
    /// ʧ���豸�����
    /// </summary>
    /// <param name="index">�豸���</param>
    /// <returns>-11:ʧ�� 0:�ɹ�</returns>
    int (*disable_phantom_forces)(unsigned int index);

    /// ���������豸
    /// </summary>
    /// <param name="index">�豸���</param>
    /// <param name="forces">��ά��ʸ��ֵ</param>
    /// <returns>-11:ʧ�� 0:�ɹ�</returns>
    int (*send_phantom_force)(unsigned int index, const float forces[3]);

    /// <summary>
    /// ȡ�豸����������һ·��
    /// XYZ ABC E1 E2 E3 Ӳ����ָ������Ҫ��������������
    /// </summary>
    /// <param name="index">�豸���</param>
    /// <param name="sEncoders">����������</param>
    /// <returns>-11:ʧ�� 0:�ɹ�</returns>
    void (*GetDeviceEncoders)(unsigned int index, unsigned short sEncoders[9]);

    /// <summary>
    /// ȡ�豸���������ڶ�·��
    /// Ӳ����ָ������Ҫ�Ǿ���ֵ������������ר����ֵ��
    /// </summary>
    /// <param name="index">�豸���</param>
    /// <param name="sEncoders">����������</param>
    /// <returns>-11:ʧ�� 0:�ɹ�</returns>
    void (*GetDeviceEncoders2)(unsigned int index, unsigned short sEncoders[14]);

    /// <summary>
    /// ��ȡ�豸״̬
    /// </summary>
    /// <param name="index">�豸���</param>
    /// <param name="cState">�豸״̬����</param>
    /// <returns>-11:ʧ�� 0:�ɹ�</returns>
    int (*ReadDeviceStates)(unsigned int index, unsigned char cState[5]);
};