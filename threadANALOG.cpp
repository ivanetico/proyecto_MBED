#include "mbed.h"
#include "rtos.h"


AnalogIn soilmois(PA_0); 

float valueSM=0.0;

Thread threadANALOG(osPriorityNormal, 512); // 1K stack size

void ANALOG_thread(); 

void ANALOG_thread() {
		while (true) {

				Thread::wait(200);
				valueSM=soilmois*100;
				  
    }
}
