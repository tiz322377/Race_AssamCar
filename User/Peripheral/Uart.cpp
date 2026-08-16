//
// Created by Administrator on 24-7-26.
//

#include "Uart.hpp"
#ifdef STM32_HAL
extern "C" {

    void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart) {

    }

}

#endif