#include "uart_comp.h"

HAL_StatusTypeDef WRP_UART_Receive_IT(UART_HandleTypeDef * huart, uint8_t * pData, uint16_t size)
{
    // ISN'T SUPPORTED FOR UART OVER USB
    return HAL_ERROR;
}