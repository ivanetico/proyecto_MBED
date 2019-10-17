#include "mbed.h"
#include "MMA8451Q.h"

#define MMA8451_I2C_ADDRESS (0x1d<<1)
Serial pc(USBTX,USBRX,19200);

void get_accel_values(float* data) {
  MMA8451Q acc(PB_9, PB_8, MMA8451_I2C_ADDRESS);
    float x;
    float y;
    float z;
        x=acc.getAccX();
        y=acc.getAccY();
        z=acc.getAccZ();
        pc.printf("x = %f\ty = %f\t z = %f\n",x,y,z);

        wait(0.2);
	float result[3] = {x,y,z};
  data = result;
		
}
