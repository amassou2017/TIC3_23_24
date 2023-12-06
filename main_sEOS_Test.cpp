#include <Arduino.h>
#include <sEOS.h>

void sEOS_Update(void)
{
  HAL_ResumeTick();//resume the systick Timer used by HAL library
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //Serial.println("Task update");
  //to preserve Energy
  HAL_SuspendTick();//halt the systick Timer used by HAL library
}

void setup(void)
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.println("Initialzing sEOS...");   
  sEOS_Init(TIM2,10,HERTZ_FORMAT,sEOS_Update);// 2 Hz
}

void loop(void)
{

}