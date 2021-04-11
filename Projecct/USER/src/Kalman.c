#include "headfile.h"
#include "Kalman.h"

/***����������***/
float PrioriErr = 0; //�������
int16 FltValue = 0; //�˲�ֵ����Ϊ�˲�����������ӽ���ʵֵ��
float KalmanGain = 0.0; //����������
float EstimateCov = 0.0; //����Э����
float MeasureCov = 0.04; //����Э����

int16 FILTRATE_Kalman(int16 MsrValue)
{
   /* static float PrioriErr = 0; //�������
    static int16 FltValue = 0; //�˲�ֵ����Ϊ�˲�����������ӽ���ʵֵ��
    static float KalmanGain = 0.0; //����������
    static float EstimateCov = 0.0; //����Э����
    static float MeasureCov = 0.20; //����Э����*/
  
    PrioriErr = EstimateCov + (float)0.05; //������� = ����Э���� + ���̷���
    //���㿨��������
    KalmanGain = PrioriErr / (PrioriErr + MeasureCov);
    //���㱾���˲�����ֵ
    //����ֵ = �ϴι���ֵ + Kalman ���� * (����ֵ - �ϴι���ֵ)
    FltValue = (int16)(FltValue + KalmanGain * (MsrValue - FltValue));
    //���¹���Э����
    EstimateCov = (float)((1-KalmanGain)*PrioriErr);
    return FltValue; //���ع���ֵ�����˲�ֵ
}

static  float Q_angle = 0.015, Q_gyro = 0.0001, R_angle = 10, dt = 0.005;
//Q���󣬶�̬��Ӧ����
static float Pk[2][2] = { {1, 0}, {0, 1 } };

static float Pdot[4] = { 0,0,0,0 };

static float q_bias = 0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

float Car_Angle,Angle_Speed,Angle_acc;


//-------------------------------------------------------
void Kalman_Filter(float angle_m, float gyro_m)
{
	Car_Angle += (gyro_m - q_bias) * dt; //Ԥ��ֵ
	Pdot[0] = Q_angle - Pk[0][1] - Pk[1][0];
	Pdot[1] = -Pk[1][1];
	Pdot[2] = -Pk[1][1];
	Pdot[3] = Q_gyro;

	Pk[0][0] += Pdot[0] * dt;
	Pk[0][1] += Pdot[1] * dt;
	Pk[1][0] += Pdot[2] * dt;
	Pk[1][1] += Pdot[3] * dt;

	angle_err = angle_m - Car_Angle;//����ֵ-Ԥ��ֵ

	PCt_0 = Pk[0][0];
	PCt_1 = Pk[1][0];

	E = R_angle + PCt_0;

	K_0 = PCt_0 / E; //����������
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = Pk[0][1];

	Pk[0][0] -= K_0 * t_0;
	Pk[0][1] -= K_0 * t_1;
	Pk[1][0] -= K_1 * t_0;
	Pk[1][1] -= K_1 * t_1;

	Car_Angle += K_0 * angle_err; //���ŽǶ�=Ԥ��ֵ+����������*(����ֵ-Ԥ��ֵ)
	q_bias += K_1 * angle_err;
	Angle_Speed = gyro_m - q_bias;
}


//�Ƕȼ������˲�
void Angle_Calculate()
{
	float ratio = 0.23;
        
	Angle_acc = (mpu_acc_z)*0.01;
	Angle_Speed = (mpu_gyro_y - 48) * ratio;
	Kalman_Filter(Angle_acc, Angle_Speed);            //���ÿ������˲�����
        
}
