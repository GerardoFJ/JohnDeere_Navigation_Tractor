/*
 * hc05_module.h
 *
 *  Created on: Sep 23, 2025
 *      Author: Gerardo Fregoso
 */

#ifndef INC_HC05_MODULE_H_
#define INC_HC05_MODULE_H_

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"


#define BUFFER_SIZE 100

void startHCrx(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void sendHC(char *data);
const char* HC05_GetData(void);

#endif /* INC_HC05_MODULE_H_ */
