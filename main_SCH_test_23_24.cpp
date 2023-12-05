#include <Arduino.h>
#include "schSTM32.h"

void task(void)
{
 digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  Serial.println("Task update");
}

void setup() {
// put your setup code here, to run once:
Serial.begin(115200);
pinMode(LED_BUILTIN,OUTPUT);
Serial.println("Initialzing SCH...");
SCH_Init(500000,MICROSEC_FORMAT);
SCH_Add_Task(task,0,2);
Serial.println("Starting SCH...");
SCH_Start();
}
//loop function
void loop() {
  SCH_Dispatch_Tasks();
}