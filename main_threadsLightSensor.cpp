#include "mbed.h"

DigitalOut myled(LED1, PullUp);
DigitalOut myled2(LED3, PullUp);
AnalogIn lightin(PA_0);
Thread thread1, thread2;

void led1_thread (void){
   while(1){
     if (lightin >= 0.8) {
      myled = 1;
    } else
        myled=0;
    wait(1);
  }
}

void led2_thread (void){
  while(1){
   if (lightin >= 0.8) {
      myled2 = 1;
    } else
        myled2=0;
    wait(1);
  }
}

int main() {

         thread1.start(led1_thread);
         thread2.start(led2_thread);

           while(1){
						 wait(1);
					 }
        }
