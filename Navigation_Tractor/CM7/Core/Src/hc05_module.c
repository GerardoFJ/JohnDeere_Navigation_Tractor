

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

void sendHC(const char *format, ...) {
    // 1. Create a buffer to hold the final formatted string
    // Adjust size based on your needs (128 bytes is usually sufficient for debug lines)
    char buffer[128];

    // 2. Initialize the list of variable arguments
    va_list args;
    va_start(args, format);

    // 3. Print formatted data into the buffer
    // vsnprintf is safer than vsprintf because it prevents buffer overflow
    vsnprintf(buffer, sizeof(buffer), format, args);

    // 4. Clean up the argument list
    va_end(args);

    // 5. Transmit via UART
    // We calculate the length of the formatted string inside the buffer
    HAL_UART_Transmit(current_huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

const char* HC05_GetData(void) {
    return hc_data;
}
