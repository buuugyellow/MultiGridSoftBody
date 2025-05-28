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

    float M[16]; // 设备笔尖控制点位姿（齐次）矩阵
    unsigned char State[5];
    unsigned short Encoders[16];
    unsigned short Encoders2[16];
    unsigned short maxValue = 0;
    unsigned short minValue = INT16_MAX;
    float m_buffer[TransSize]; // 整合位姿矩阵和夹钳角度

    void Init(string name);
    void Stop();

    /// <summary>
    /// 建立服务循环
    /// </summary>
    /// <returns>0: 成功 -1: 失败</returns>
    int (*createServoLoop)();

    /// <summary>
    /// 停止服务循环
    /// </summary>
    /// <returns>0: 成功 -1: 失败</returns>
    int (*stopServoLoop)();

    /// <summary>
    /// 终止服务循环
    /// </summary>
    /// <returns>0: 失败 1:成功</returns>
    int (*destroyServoLoop)();

    /// <summary>
    /// 初始化设备
    /// </summary>
    /// <param name="configname">设备名称（与配置文件同名）</param>
    /// <returns>-10: 失败，>=0: 设备索引号</returns>
    int (*init_phantom)(const char* configname);

    /// <summary>
    /// 使能设备
    /// </summary>
    /// <param name="index">设备序号</param>
    /// <returns>-11:失败 0:成功</returns>
    int (*disable_phantom)(unsigned int index);

    /// <summary>
    /// 启动服务循环
    /// </summary>
    /// <param name="func">回调函数指针</param>
    /// <param name="lpParam">回调函数参数指针</param>
    /// <returns>-11: 回调函数指针为NULL -1:失败或服务循环已启动 0:启动成功</returns>
    int (*startServoLoop)(int(_stdcall* fntServo)(void*), void* lpParam);

    /// <summary>
    /// 取设备笔尖控制点位姿（齐次）矩阵
    /// </summary>
    /// <param name="index">设备序号</param>
    /// <param name="matrix">位姿（齐次）矩阵</param>
    /// <returns>-11:失败 0:成功</returns>
    int (*get_stylus_matrix)(unsigned int index, float (*matrix)[16]);

    /// <summary>
    /// 更新设备
    /// </summary>
    /// <param name="index">设备序号</param>
    /// <returns>-11:失败 0:成功 其他：设备状态值</returns>
    int (*update_phantom)(unsigned int index);

    /// <summary>
    /// 设备力操作使能
    /// </summary>
    /// <param name="index">设备序号</param>
    /// <returns>-11:失败 0:成功</returns>
    int (*enable_phantom_forces)(unsigned int index);

    /// <summary>
    /// 失能设备力输出
    /// </summary>
    /// <param name="index">设备序号</param>
    /// <returns>-11:失败 0:成功</returns>
    int (*disable_phantom_forces)(unsigned int index);

    /// 发送力到设备
    /// </summary>
    /// <param name="index">设备序号</param>
    /// <param name="forces">三维力矢量值</param>
    /// <returns>-11:失败 0:成功</returns>
    int (*send_phantom_force)(unsigned int index, const float forces[3]);

    /// <summary>
    /// 取设备编码器（第一路）
    /// XYZ ABC E1 E2 E3 硬件上指定（主要是增量编码器）
    /// </summary>
    /// <param name="index">设备序号</param>
    /// <param name="sEncoders">编码器数组</param>
    /// <returns>-11:失败 0:成功</returns>
    void (*GetDeviceEncoders)(unsigned int index, unsigned short sEncoders[9]);

    /// <summary>
    /// 取设备编码器（第二路）
    /// 硬件上指定（主要是绝对值编码器或其它专用数值）
    /// </summary>
    /// <param name="index">设备序号</param>
    /// <param name="sEncoders">编码器数组</param>
    /// <returns>-11:失败 0:成功</returns>
    void (*GetDeviceEncoders2)(unsigned int index, unsigned short sEncoders[14]);

    /// <summary>
    /// 读取设备状态
    /// </summary>
    /// <param name="index">设备序号</param>
    /// <param name="cState">设备状态数组</param>
    /// <returns>-11:失败 0:成功</returns>
    int (*ReadDeviceStates)(unsigned int index, unsigned char cState[5]);
};