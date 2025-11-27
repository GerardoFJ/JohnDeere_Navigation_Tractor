

#include "hc05_module.h"


uint8_t rx2_byte;
char hc_data[BUFFER_SIZE];
char data_colector[BUFFER_SIZE];
uint8_t data_index = 0;
static UART_HandleTypeDef *current_huart = NULL;

void startHCrx(UART_HandleTypeDef *huart) {
    current_huart = huart;
    HAL_UART_Receive_IT(current_huart, &rx2_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
    	if(rx2_byte == '\r' || rx2_byte == '\n'){
    		data_colector[data_index] = '\0';
    		strcpy(hc_data, data_colector);
//    		printf("Data saved = %s data len de %i \n\r", hc_data, strlen(hc_data));
    		data_index = 0;
    	}
    	else{
    		if(data_index < BUFFER_SIZE - 1){
    		data_colector[data_index++] = rx2_byte;
    		}
    	}
    	HAL_UART_Receive_IT(current_huart, &rx2_byte, 1);
    }
}

void sendHC(char *data) {
    if (data == NULL) return;
    HAL_UART_Transmit(current_huart, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
}

const char* HC05_GetData(void) {
    return hc_data;
}
