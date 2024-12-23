//
// Created by CYK on 2024/12/1.
//

#ifndef SERVO_CTRL_TASK_H
#define SERVO_CTRL_TASK_H

#ifdef __cplusplus
extern "C"
{
#endif

void Servo_Ctrl_Task(void *argument);
void Servo_Ctrl_UartTxCpltCallBack(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif //SERVO_CTRL_TASK_H
