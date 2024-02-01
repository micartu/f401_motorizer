#include "remote.h"
#include <string.h>

// needed implementations which control fake's behavior:
extern "C"
{
    /// callback if data over UART is available
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
    {
        // calls needed callbacks for us
        uart_irq_handler(huart);
    }
}