

#include "motor_controller.h"

static TIM_HandleTypeDef *current_tim = NULL;
static TIM_HandleTypeDef *current_tim_servo = NULL;


void startMotor(TIM_HandleTypeDef *timnum) {
	current_tim = timnum;
	 HAL_TIM_PWM_Start(current_tim, TIM_CHANNEL_1);
	 __HAL_TIM_SET_COMPARE(current_tim, TIM_CHANNEL_1, (uint32_t)INITIAL_STATE);

}
void startServo(TIM_HandleTypeDef *tim_servo) {
	current_tim_servo = tim_servo;
	 HAL_TIM_PWM_Start(current_tim_servo, TIM_CHANNEL_1);
	 __HAL_TIM_SET_COMPARE(current_tim_servo, TIM_CHANNEL_1, (uint32_t)SERVO_STATE);

}

void setMotorStep(int step){
	step = step + INITIAL_STATE;
	__HAL_TIM_SET_COMPARE(current_tim, TIM_CHANNEL_1, (uint32_t)step);
}

void setServo(int step){
	step = step + SERVO_STATE;
	__HAL_TIM_SET_COMPARE(current_tim_servo, TIM_CHANNEL_1, (uint32_t)step);
}
