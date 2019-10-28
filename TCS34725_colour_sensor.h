#include "mbed.h"

class ColourSensor{
	public:
		ColourSensor(I2C *i2c, DigitalOut *led);
		void getCRGB(unsigned short ** CRGB_values);
	private:
		I2C *mI2C;
		DigitalOut *mLed;
		void initialize_colour_sensor(void);
};