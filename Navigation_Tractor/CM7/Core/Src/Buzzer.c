#include "Buzzer.h"

int melody[] = {
  NOTE_E5, NOTE_D5, NOTE_FS4, NOTE_GS4,
  NOTE_CS5, NOTE_B4, NOTE_D4, NOTE_E4,
  NOTE_B4, NOTE_A4, NOTE_CS4, NOTE_E4,
  NOTE_A4
};

int durations[] = {
  8, 8, 4, 4,
  8, 8, 4, 4,
  8, 8, 4, 4,
  2
};
int melodysize = (sizeof(melody)/sizeof(melody[0]));

void startBuzzer(TIM_HandleTypeDef *htim){
	current_htim = htim;
	 HAL_TIM_PWM_Start(current_htim, TIM_CHANNEL_1);
	 __HAL_TIM_SET_COMPARE(current_htim, TIM_CHANNEL_1,50);
}

int presForFrequency(int Frequency){
	if(Frequency == 0) return 0;
	return((TIM_FREQ/(Frequency*100))-1);

}

void noTone(void){
	__HAL_TIM_SET_PRESCALER(current_htim,9);
}

void playTone(int *tone, int *duration, int size){
	for (int i = 0; i < size; i++){
		int pres = 9;
		if(tone[i] != 0) pres = presForFrequency(tone[i]);
		int dur = 1000 / duration[i];
		__HAL_TIM_SET_PRESCALER(current_htim,pres);
		osDelay(dur);
		noTone();
		osDelay(dur * 0.7);
	}
	noTone();
}
