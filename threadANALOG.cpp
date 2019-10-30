#include "mbed.h"
#include "types.h"
AnalogIn soilmois(PA_0); 
AnalogIn light(PA_4);

extern int time_interval;
extern int mode;
extern float valueSM;
extern float valueLight;
extern bool alarm_triggered[];

Thread thread_analog(osPriorityNormal, 512); // 1K stack size

void read_analog(void) {
  while (true) {

    valueSM = soilmois*100;
	if(valueSM > 60.0)
		alarm_triggered[soilmoistureAlarm] = true;
	else
		alarm_triggered[soilmoistureAlarm] = false;
	
    valueLight = light*100;
	if(valueLight > 60.0)
		alarm_triggered[ambientLightAlarm] = true;
	else
		alarm_triggered[ambientLightAlarm] = false;
	
	if(mode==0)
			Thread::wait(800);
	else
		 Thread::wait(((float)time_interval/2.5)*1000);
	}
}
