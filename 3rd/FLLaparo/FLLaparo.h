#pragma once

#ifdef FLLAPARO_EXPORTS
#define FLLAPARO_API __declspec(dllexport)
#else
#define FLLAPARO_API __declspec(dllimport)
#endif
namespace fllaparo {

	extern "C"
	{
		//���豸Ϊ�豸���к�
		//���� sn���豸���к�
		//���� �豸ID
		FLLAPARO_API int openDevice(DWORD sn);
		//��ͣ�豸
		FLLAPARO_API void PauseDevice(int id = 0);
		//�ָ��豸
		FLLAPARO_API void ResumeDevice(int id = 0);
		//�ر��豸
		FLLAPARO_API void closeDevice();
		//��ʼ����ѭ��
		FLLAPARO_API int startServoLoop(int(_stdcall* func)(void*), void* lpParam);
		//ֹͣ����ѭ��
		FLLAPARO_API int stopServoLoop();
		//�жϷ���ѭ���Ƿ���ִ��
		FLLAPARO_API bool isServoLoopRunning();
		//ȡ����ѭ��ˢ����
		FLLAPARO_API double getServoLoopRate(int id = 0);
		//�豸��ʹ�ܺ��� true ʹ�ܣ�false ��ʹ��
		FLLAPARO_API void enableForces(bool en = true, int id = 0);
		//�ж��豸�Ƿ�����ʹ��
		FLLAPARO_API bool isForcesEnabled(int id = 0);
		//����豸���к�
		FLLAPARO_API int getSerialNumber(int id = 0);
		//�豸״̬����δʵ�֣�
		FLLAPARO_API int deviceStatus(int id = 0);
		//���������غ���
		FLLAPARO_API void sendZeroForce(int id = 0);
		//��������ǯ�������
		FLLAPARO_API void sendForce(double force[], int id = 0);
		//���ø�������ֵΪ32768
		FLLAPARO_API void zeroEncoders(int id = 0);
		//ȡ���������ֵ
		FLLAPARO_API void getEncoders(long encs[], int id = 0);
		//ȡ����������ٶ�
		FLLAPARO_API void getEncVel(double evels[], int id = 0);
		//ȡ������״̬  
		//bit7:button1  bit5:button2
		FLLAPARO_API void getSwitch(byte& swts, int id = 0);
		//ȡ�ؽ�ֵ
		//joints: �ؽ�����
		FLLAPARO_API void getJoints(double joints[], int id = 0);
		//ȡ����ǯ�������ϵ  
		FLLAPARO_API void getEndFrame(double EndFrame[], int id = 0);
		//ȡ����ǯ���λ�á���ת�Ƕȡ�ǯ�ڽǶ�
		FLLAPARO_API void getPositionAndAngle(double pos[], double& rotAngle, double& Angle, int id = 0);
		//ȡ����ǯ�����̬
		FLLAPARO_API void getPose(double rot[], int id = 0);

	}
}