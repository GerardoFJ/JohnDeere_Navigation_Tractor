

#include "motor_controller.h"

static TIM_HandleTypeDef *current_tim = NULL;


void startMotor(TIM_HandleTypeDef *timnum) {
	current_tim = timnum;
	 HAL_TIM_PWM_Start(current_tim, TIM_CHANNEL_1);
	 __HAL_TIM_SET_COMPARE(current_tim, TIM_CHANNEL_1, (uint32_t)INITIAL_STATE);

}

void setMotorStep(uint32_t step){
	__HAL_TIM_SET_COMPARE(current_tim, TIM_CHANNEL_1, (uint32_t)step);
}
