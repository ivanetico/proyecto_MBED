#include "mbed.h"
#include "types.h"

extern int mode;
Thread thread_alarms(osPriorityNormal, 512);

extern DigitalOut ledR; //RGB led - red light
extern DigitalOut ledG;  //RGB led - green light 
extern DigitalOut ledB;  //RGB led - blue light

bool alarm_triggered[6];

void trigger_alarm(void){
	memset(alarm_triggered, 0, sizeof(alarm_triggered));
	
	while(mode!=0){
		bool something_triggered = false;
		for(int i = temperatureAlarm; i<=accelerationAlarm; i++){
			if(alarm_triggered[i])
				switch(i){
					case temperatureAlarm:
						something_triggered = true;
						ledR = 0;
						ledG = 1;
						ledB = 1;
						Thread::wait(400);
						break;
					case humidityAlarm:
						something_triggered = true;
						ledR = 1;
						ledG = 1;
						ledB = 0;
						Thread::wait(400);
						break;
					case ambientLightAlarm:
						something_triggered = true;
						ledR = 0;
						ledG = 0;
						ledB = 1;
						Thread::wait(400);
						break;
					case soilmoistureAlarm:
						something_triggered = true;
						ledR = 0;
						ledG = 1;
						ledB = 0;
						Thread::wait(400);
						break;
					case colourAlarm:
						something_triggered = true;
						ledR = 1;
						ledG = 0;
						ledB = 1;
						Thread::wait(400);
						break;
					case accelerationAlarm:
						something_triggered = true;
						ledR = 1;
						ledG = 0;
						ledB = 0;
						Thread::wait(400);
						break;		
					}
		}
		if(!something_triggered){
				ledR = 1;
				ledG = 1;
				ledB = 1;
				Thread::wait(400); //This forces the thread to wait at least one time
		}
	}

}