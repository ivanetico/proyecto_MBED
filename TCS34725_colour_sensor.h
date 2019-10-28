#include "mbed.h"

class ColourSensor{
	public:
		ColourSensor(I2C *i2c, DigitalOut *led);
		void getCRGB(uint16_t * CRGB_values);
	private:
		I2C *mI2C;
		DigitalOut *mLed;
		void initialize_colour_sensor(void);
};