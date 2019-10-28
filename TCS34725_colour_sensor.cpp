#include "TCS34725_colour_sensor.h"

// We set the sensor address. For TCS34725 is 0x29 = ??0010 1001? (bin) ->> ?0101 0010? (bin) = 0x52
// We shift 1 bit to the left because in I2C protocol slave address is 7-bit. So we discard the 8th bit
int colour_sensor_addr = 0x29 << 1; 

ColourSensor::ColourSensor(I2C *i2c, DigitalOut *led){
	mI2C = i2c;
	mLed = led;
	initialize_colour_sensor();
}
void ColourSensor::getCRGB(uint16_t * CRGB_values){
		// Read data from color sensor (Clear/Red/Green/Blue)
		char colour_data[8];
		uint16_t *CRGB_values_aux;
		char clear_reg[1] = {0xB4}; // {?1011 0100?} -> 0x14 and we set 1st and 3rd bit to 1 for auto-increment
		//Asking for first register value
		mI2C->write(colour_sensor_addr,clear_reg, 1, true);
		mI2C->read(colour_sensor_addr,colour_data, 8, false);
		
		CRGB_values_aux = (uint16_t*)(void*)colour_data;
		memcpy(CRGB_values, CRGB_values_aux, sizeof(uint16_t)*4);

}
void ColourSensor::initialize_colour_sensor(){
	char id_regval[1] = {0x92}; //?1001 0010? (bin)
  char data[1] = {0}; //?0000 0000?
	*mLed=1;
	/**********************************************************************
	* int write(int address, const char *data, int length, bool repeated) *
	* int read(int address, char *data, int length, bool repeated)        *
	***********************************************************************/
	//We obtain device ID from ID register (0x12)
    mI2C->write(colour_sensor_addr,id_regval,1, true);
    mI2C->read(colour_sensor_addr,data,1,false); 
   
    
    // Initialize color sensor
    
	// Timing register address 0x01 (0000 0001). We set 1st bit to 1 -> 1000 0001
    char timing_register[2] = {0x81,0x50}; //0x50 ~ 400ms
    mI2C->write(colour_sensor_addr,timing_register,2,false); 
    
	// Control register address 0x0F (0000 1111). We set 1st bit to 1 -> 1000 1111
    char control_register[2] = {0x8F,0}; //{0x8F, 0x00}, {1000 1111, 0000 0000} -> 1x gain
    mI2C->write(colour_sensor_addr,control_register,2,false);
    
	// Enable register address 0x00 (0000 0000). We set 1st bit to 1 -> 1000 0000
    char enable_register[2] = {0x80,0x03}; //{0x80, 0x03}, {1000 0000, 0000 0011} -> AEN = PON = 1
    mI2C->write(colour_sensor_addr,enable_register,2,false);
    
}