#include <Arduino.h>

void sEOS_Init(TIM_TypeDef *Instance, uint32_t intervall, TimerFormat_t format, callback_function_t callback)
{
  HardwareTimer *MyTim = new HardwareTimer(Instance);// declare object MyTim
 
  MyTim->setOverflow(intervall, format); 
  MyTim->attachInterrupt(callback);//configure the function callback as an interrupt for the Update event
  MyTim->resume();// start Timer
  HAL_SuspendTick();// Halt systick timer used by HAL library
  HAL_PWR_EnableSleepOnExit();// Sleep after intterupt 
}
