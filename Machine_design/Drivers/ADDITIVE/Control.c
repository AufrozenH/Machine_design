#include "Control.h"
#include "tim.h"
#include "stdint.h"
#include "cmsis_os.h"
/********************************************************************结构体变量*******************************************************************************/
PID x_dir; // 二维云台X方向
PID y_dir; // 二维云台Y方向
//=====================================================常规函数区=====================================================
//-------------------------------------------------------------------------------------------------------------------
// @brief       SE_init()
// @param       void
// @return      void
// @function    外设初始化（舵机、电机、步进电机、推杆）
//-------------------------------------------------------------------------------------------------------------------
void SE_init(void)
{
	// 舵机初始化
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // TIM1通道1PWM初始化
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // TIM1通道2PWM初始化
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // TIM1通道3PWM初始化(后面下层舵机)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // TIM1通道4PWM初始化(前面上层舵机)
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // TIM2通道1PWM初始化
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // TIM2通道2PWM初始化(后面下层舵机)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // TIM3通道1PWM初始化()
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // TIM3通道2PWM初始化
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // TIM3通道3PWM初始化（顶落种子舵机）
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // TIM3通道4PWM初始化
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); // TIM9通道1PWM初始化(多余的测试口)
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); // TIM9通道2PWM初始化(前面下层舵机)
	// 步进电机初始化
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // TIM4通道2PWM初始化（右传送带）
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // TIM4通道4PWM初始化（左传送带）
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // TIM8通道2PWM初始化（第一个播种传送带）
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); // TIM8通道4PWM初始化（第二个播种传送带）
	// 步进电机初始化
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  // TIM2通道3PWM初始化
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // TIM2通道4PWM初始化
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // TIM12通道1PWM初始化
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // TIM12通道2PWM初始化(第一个落土电机)
	// 落盘舵机归位
	FU_DJPWM(0);
	FD_DJPWM(0);
	BU_DJPWM(0);
	BD_DJPWM(0);
	// 落种子舵机归位
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 180);
	//刮图舵机归位
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 190);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       LP_start(void)
// @param       void
// @return      void
// @function    实现第一次落盘
//-------------------------------------------------------------------------------------------------------------------
void LP_start(void)
{
	// 落盘上层舵机打开
	FU_DJPWM(1);
	BU_DJPWM(1);
	osDelay(1000);
	// 落盘上层舵机关闭
	FU_DJPWM(0);
	BU_DJPWM(0);
	osDelay(1000);
	// 落盘下层舵机打开
	FD_DJPWM(1);
	BD_DJPWM(1);
	osDelay(1000);
	// 落盘下层舵机关闭
	FD_DJPWM(0);
	BD_DJPWM(0);
	osDelay(1000);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       CSD_Move(uint8_t state)
// @param       state 0:停止状态 1:运动状态
// @return      void
// @function    实现传送带运动
//-------------------------------------------------------------------------------------------------------------------
void CSD_Move(uint8_t state)
{
	switch(state)
	{
	    case 0:	
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0); 
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); 
			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 50); 
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 50);
			break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       ZZ_start(void)
// @param       void
// @return      void
// @function    实现种子下落
//-------------------------------------------------------------------------------------------------------------------
void ZZ_start(void)
{
	ZZ_PWM(2);
	osDelay(19000);
	ZZ_PWM(0);
	osDelay(500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 270);
	osDelay(1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 180);
	osDelay(1000);
	ZZ_PWM(1);
	osDelay(19000);
	ZZ_PWM(0);
	osDelay(500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 270);
	osDelay(1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 180);
	osDelay(500);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       FU_DJPWM(uint8_t state)
// @param       state 0:关闭状态  1：打开状态
// @return      void
// @function    落盘前面上层舵机pwm输出
//-------------------------------------------------------------------------------------------------------------------
void FU_DJPWM(uint8_t state)
{
	switch (state)
	{
	case 0:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 120);
		break;
	case 1:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 160);
		break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       FD_DJPWM(uint8_t state)
// @param       state 0:关闭状态  1：打开状态
// @return      void
// @function    落盘前面下层舵机pwm输出
//-------------------------------------------------------------------------------------------------------------------
void FD_DJPWM(uint8_t state)
{
	switch (state)
	{
	case 0:
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 130);
		break;
	case 1:
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 90);
		break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       BU_DJPWM(uint8_t state)
// @param       state 0:关闭状态  1：打开状态
// @return      void
// @function    落盘后面上层舵机pwm输出
//-------------------------------------------------------------------------------------------------------------------
void BU_DJPWM(uint8_t state)
{
	switch (state)
	{
	case 0:
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 170);
		break;
	case 1:
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 210);
		break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       BD_DJPWM(uint8_t state)
// @param       state 0:关闭状态  1：打开状态
// @return      void
// @function    落盘后面下层舵机pwm输出
//-------------------------------------------------------------------------------------------------------------------
void BD_DJPWM(uint8_t state)
{
	switch (state)
	{
	case 0:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 200);
		break;
	case 1:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 165);
		break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       ZZ_PWM(uint8_t state)
// @param       state  0: 停止状态 1:后退状态  2：前进状态
// @return      void
// @function    落盘pwm输出
//-------------------------------------------------------------------------------------------------------------------
void ZZ_PWM(uint8_t state)
{
	switch (state)
	{
	case 0:
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 50);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 50);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 50);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 50);
		break;
		break;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       pwm()
// @param       void
// @return      void
// @function    二维云台舵机的pwm脉冲信号输出
//-------------------------------------------------------------------------------------------------------------------
void pwm(int xpwm, int ypwm)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, xpwm);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ypwm);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       PID_init()
// @param       void
// @return      void
// @function    云台舵机的参数初始化
//-------------------------------------------------------------------------------------------------------------------
void PID_init(PID *x, float P, float I, float D)
{
	x->now_error = 0;
	x->pre_error = 0;
	x->d_error = 0;
	x->sum_error = 0;
	x->P = P;
	x->I = I;
	x->D = D;
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       PID_init()
// @param       void
// @return      void
// @function    云台舵机的pwm输出
//-------------------------------------------------------------------------------------------------------------------
float Position_pid(float target, float reality, PID *pid) // 位置式PID
{
	pid->now_error = target - reality; // 谁减谁需要调整
	pid->d_error = pid->now_error - pid->pre_error;
	pid->sum_error += pid->now_error;

	pid->output = (pid->P * pid->now_error) + (pid->D * pid->d_error) + (pid->I * pid->sum_error);

	pid->pre_error = pid->now_error;

	return pid->output;
}