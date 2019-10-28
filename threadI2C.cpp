#include "mbed.h"
#include "TCS34725_colour_sensor.h"
#include "MMA8451Q.h"
#define MMA8451_I2C_ADDRESS (0x1c<<1)
#define tempHumAddres (0x40<<1)

I2C i2c(I2C_SDA, I2C_SCL); //pins for I2C communication (SDA, SCL)
DigitalOut ledColour(PB_7); // TCS34725 led
ColourSensor colourSensor(&i2c, &ledColour);
DigitalOut ledR(PH_0, PullDown); //RGB led - red light
DigitalOut ledG(PH_1, PullDown);  //RGB led - green light 
DigitalOut ledB(PB_13, PullDown);  //RGB led - blue light
MMA8451Q acc(I2C_SDA, I2C_SCL, MMA8451_I2C_ADDRESS);

extern uint16_t CRGB_values[];
extern float accel_data[];
extern float temp, hum;
extern char colour[];
extern int mode;
Thread thread_i2c(osPriorityNormal, 1024);

void shutDownLed(void){
  ledR.write(1);
  ledG.write(1);
  ledB.write(1);
}
//Get max value (r,g,b) function
void setLEDColour(unsigned short *CRGB_value) {

  shutDownLed();

  if (CRGB_value[1] > CRGB_value[2] && CRGB_value[1] > CRGB_value[3]){
    ledR.write(0);
    strcpy(colour,"RED  ");
  }else if(CRGB_value[2] > CRGB_value[1] && CRGB_value[2] > CRGB_value[3]){
    ledG.write(0);
    strcpy(colour,"GREEN");
  }else if (CRGB_value[3] > CRGB_value[1] && CRGB_value[3] > CRGB_value[2]){
    ledB.write(0);
    strcpy(colour,"BLUE ");
  }
}
	//Switchs the color of the greatest value. First, we switch off all of them

void get_TempHum_values(float* temp, float* hum){
	char tx[1]={0xE3};
	char rx[2];
	
	i2c.write(tempHumAddres, tx, 1, true);
	i2c.read(tempHumAddres, rx, 2, false);
	*temp = (((rx[0]<<8) + rx[1])*175.72/65536.0) - 46.85;
	
	tx[0]=0xE5;
	i2c.write(tempHumAddres, tx, 1, true);
	i2c.read(tempHumAddres, rx, 2, false);
	*hum = (((rx[0]<<8) + rx[1])*125.0/65536.0) - 6.0;
}

void read_i2c(void){
	while(true){
		//We store the values read in clear, red, green and blue data
		colourSensor.getCRGB(CRGB_values);					
		//Sets LED higuer colour
			if (mode == 0)
        setLEDColour(CRGB_values);
    acc.getAccAllAxis(accel_data);
		get_TempHum_values(&temp,&hum);
  if(mode==0)
      Thread::wait(1000);
  else
     Thread::wait(15000);
  }
}