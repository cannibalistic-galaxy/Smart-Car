#include "headfile.h"
#include "Kalman.h"

/***卡尔曼参数***/
float PrioriErr = 0; //先验误差
int16 FltValue = 0; //滤波值，作为滤波器输出，更接近真实值。
float KalmanGain = 0.0; //卡尔曼增益
float EstimateCov = 0.0; //估计协方差
float MeasureCov = 0.04; //测量协方差

int16 FILTRATE_Kalman(int16 MsrValue)
{
   /* static float PrioriErr = 0; //先验误差
    static int16 FltValue = 0; //滤波值，作为滤波器输出，更接近真实值。
    static float KalmanGain = 0.0; //卡尔曼增益
    static float EstimateCov = 0.0; //估计协方差
    static float MeasureCov = 0.20; //测量协方差*/
  
    PrioriErr = EstimateCov + (float)0.05; //先验误差 = 估计协方差 + 过程方差
    //计算卡尔曼增益
    KalmanGain = PrioriErr / (PrioriErr + MeasureCov);
    //计算本次滤波估计值
    //估计值 = 上次估计值 + Kalman 增益 * (测量值 - 上次估计值)
    FltValue = (int16)(FltValue + KalmanGain * (MsrValue - FltValue));
    //更新估计协方差
    EstimateCov = (float)((1-KalmanGain)*PrioriErr);
    return FltValue; //返回估计值，即滤波值
}

static  float Q_angle = 0.015, Q_gyro = 0.0001, R_angle = 10, dt = 0.005;
//Q增大，动态响应增大
static float Pk[2][2] = { {1, 0}, {0, 1 } };

static float Pdot[4] = { 0,0,0,0 };

static float q_bias = 0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

float Car_Angle,Angle_Speed,Angle_acc;


//-------------------------------------------------------
void Kalman_Filter(float angle_m, float gyro_m)
{
	Car_Angle += (gyro_m - q_bias) * dt; //预测值
	Pdot[0] = Q_angle - Pk[0][1] - Pk[1][0];
	Pdot[1] = -Pk[1][1];
	Pdot[2] = -Pk[1][1];
	Pdot[3] = Q_gyro;

	Pk[0][0] += Pdot[0] * dt;
	Pk[0][1] += Pdot[1] * dt;
	Pk[1][0] += Pdot[2] * dt;
	Pk[1][1] += Pdot[3] * dt;

	angle_err = angle_m - Car_Angle;//测量值-预测值

	PCt_0 = Pk[0][0];
	PCt_1 = Pk[1][0];

	E = R_angle + PCt_0;

	K_0 = PCt_0 / E; //卡尔曼增益
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = Pk[0][1];

	Pk[0][0] -= K_0 * t_0;
	Pk[0][1] -= K_0 * t_1;
	Pk[1][0] -= K_1 * t_0;
	Pk[1][1] -= K_1 * t_1;

	Car_Angle += K_0 * angle_err; //最优角度=预测值+卡尔曼增益*(测量值-预测值)
	q_bias += K_1 * angle_err;
	Angle_Speed = gyro_m - q_bias;
}


//角度计算与滤波
void Angle_Calculate()
{
	float ratio = 0.23;
        
	Angle_acc = (mpu_acc_z)*0.01;
	Angle_Speed = (mpu_gyro_y - 48) * ratio;
	Kalman_Filter(Angle_acc, Angle_Speed);            //调用卡尔曼滤波函数
        
}
